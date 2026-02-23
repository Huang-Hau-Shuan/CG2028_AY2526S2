/******************************************************************************
 * @file    sound_sensor.c
 * @brief   Grove Sound Sensor driver — ADC sampling, threshold detection,
 *          and audio recording with UART streaming.
 *
 * Pin:  PC5  (Arduino A0)  →  ADC1 channel 14
 * Timer: TIM3 generates periodic triggers for DMA-based recording.
 * DMA:  DMA1 Channel 1 (Request 0) feeds ADC1 conversions into a
 *        circular double-buffer.
 *
 * HOW IT WORKS
 * ────────────
 *   Threshold mode:
 *     Sound_ReadADC()        → single blocking conversion (polling).
 *     Sound_ThresholdCheck()  → called each main-loop tick; compares
 *                               reading against SOUND_THRESH_HIGH.
 *
 *   Recording mode:
 *     Sound_StartRecording()  → starts TIM3 + ADC + DMA in circular mode.
 *     DMA half-complete / complete interrupts set flags.
 *     Sound_RecordingTick()   → checks flags, streams the ready half-buffer
 *                               over UART as raw bytes (or WiFi stub).
 *     Sound_StopRecording()   → halts everything; sets state to IDLE.
 *
 * UART BINARY PROTOCOL  (when SOUND_STREAM_OVER_UART == 1)
 * ──────────────────────
 *   Each chunk transmitted:
 *     [0xAA] [0x55]                ← 2-byte sync header
 *     [len_hi] [len_lo]           ← number of samples in this chunk (big-endian)
 *     [sample0_hi] [sample0_lo]   ← 12-bit ADC values stored in uint16_t
 *     ...                          ← (len) total sample pairs
 *
 *   When SOUND_DEBUG_PRINT == 1, human-readable status lines are also sent
 *   (prefixed with "#" so a receiver script can filter them out).
 *
 * SAVING TO PC
 * ─────────────
 *   On your laptop, open a serial terminal in raw/binary logging mode
 *   (e.g. PuTTY → Session → Logging → "All session output", or use a
 *    Python script with pyserial to write to a .raw file).
 *   The captured binary can be imported into Audacity as:
 *     Format: Signed 16-bit PCM, Little-endian, 1 channel, 8000 Hz.
 *   (Strip the 4-byte headers from each chunk first, or write a simple
 *    Python parser.)
 *
 * @note   System clock assumed 4 MHz (MSI default, no PLL).
 ******************************************************************************/

#include "sound_sensor.h"
#include "string.h"
#include "stdio.h"

/*============================================================================
 *  PRIVATE VARIABLES
 *============================================================================*/

/* ADC & DMA handles (private to this file) */
static ADC_HandleTypeDef hadc1;
static DMA_HandleTypeDef hdma_adc1;

/* TIM handle for periodic ADC triggering during recording */
static TIM_HandleTypeDef htim3;

/* UART handle — points to the handle passed in Sound_Init() */
static UART_HandleTypeDef *pUart = NULL;

/* ---- DMA double-buffer ----
 * Total size = 2 × SOUND_DMA_HALF_BUF_SIZE uint16_t values.
 * DMA fills this circularly; half-transfer and transfer-complete
 * interrupts tell us which half is ready for consumption.
 */
static uint16_t dma_audio_buf[2 * SOUND_DMA_HALF_BUF_SIZE];

/* Flags set by DMA IRQ callbacks, consumed by Sound_RecordingTick() */
static volatile uint8_t dma_half_ready = 0;   /* 1 = first half ready  */
static volatile uint8_t dma_full_ready = 0;   /* 1 = second half ready */

/* Recording state */
static Sound_RecState_t rec_state = SOUND_REC_IDLE;
static uint32_t rec_start_tick    = 0;     /* HAL_GetTick() at start   */
static uint32_t rec_samples_sent  = 0;     /* total samples streamed   */

/* ---- Threshold detection state ---- */
static uint16_t thresh_consec_count = 0;   /* consecutive above-thresh */
static uint16_t thresh_cooldown     = 0;   /* remaining cooldown ticks */
static uint16_t thresh_peak         = 0;   /* peak during current run  */

/* Small text buffer for debug prints */
static char dbg_buf[128];

/*============================================================================
 *  FORWARD DECLARATIONS  (private helpers)
 *============================================================================*/
static void ADC1_Init(void);
static void TIM3_Init(void);
static void DMA1_Init(void);
static void Sound_StreamHalf(const uint16_t *buf, uint16_t len);

#if SOUND_DEBUG_PRINT
static void Sound_DbgPrint(const char *msg);
#endif

/*============================================================================
 *  PUBLIC FUNCTIONS
 *============================================================================*/

/* -------------------------------------------------------------------------
 * Sound_Init
 * -------------------------------------------------------------------------
 * Initialise ADC1 on PC5, optionally TIM3 + DMA for recording.
 * Call after HAL_Init().
 * ------------------------------------------------------------------------- */
void Sound_Init(UART_HandleTypeDef *huart)
{
    pUart = huart;

    /* 1. Configure PC5 as analog input (no pull, analog mode) */
    __HAL_RCC_GPIOC_CLK_ENABLE();                  /* Enable GPIOC clock   */
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin  = GPIO_PIN_5;                         /* PC5 = ARD_A0         */
    gpio.Mode = GPIO_MODE_ANALOG_ADC_CONTROL;       /* Analog for ADC       */
    gpio.Pull = GPIO_NOPULL;                        /* No pull-up/down      */
    HAL_GPIO_Init(GPIOC, &gpio);

    /* 2. Initialise ADC1 */
    ADC1_Init();

#if SOUND_ENABLE_RECORDING
    /* 3. Initialise DMA for ADC (must be before TIM so IRQs are ready) */
    DMA1_Init();

    /* 4. Initialise TIM3 for periodic trigger */
    TIM3_Init();
#endif

#if SOUND_DEBUG_PRINT
    Sound_DbgPrint("# Sound_Init(): Grove sensor on PC5 (A0) ready\r\n");
#endif
}

/* -------------------------------------------------------------------------
 * Sound_ReadADC  — single blocking read
 * -------------------------------------------------------------------------
 * Configures the channel, starts a conversion, waits, returns 12-bit value.
 * Safe to call from the main loop (not from ISR).
 * ------------------------------------------------------------------------- */
uint16_t Sound_ReadADC(void)
{
    /* Select ADC1 channel 14 (PC5) for regular conversion */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_14;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;  /* ~12 µs at 4 MHz */
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
        return 0;

    /* Start conversion, wait, read */
    HAL_ADC_Start(&hadc1);
    if (HAL_ADC_PollForConversion(&hadc1, 10) != HAL_OK)
    {
        HAL_ADC_Stop(&hadc1);
        return 0;
    }
    uint16_t val = (uint16_t)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

/* -------------------------------------------------------------------------
 * Sound_ThresholdCheck
 * -------------------------------------------------------------------------
 * Call once per main-loop iteration.
 * Reads ADC, applies consecutive-count + cooldown logic.
 * Sets evt->detected = 1 on a new loud-sound event.
 * ------------------------------------------------------------------------- */
void Sound_ThresholdCheck(Sound_ThreshEvent_t *evt)
{
#if !SOUND_ENABLE_THRESHOLD
    evt->detected     = 0;
    evt->peak_adc     = 0;
    evt->timestamp_ms = 0;
    return;
#else
    evt->detected = 0;     /* default: no event this tick */

    /* --- Cooldown handling --- */
    if (thresh_cooldown > 0)
    {
        thresh_cooldown--;
        return;              /* still cooling down — skip check */
    }

    /* --- Read current sound level --- */
    uint16_t adc_val = Sound_ReadADC();

    if (adc_val >= SOUND_THRESH_HIGH)
    {
        /* Track peak during this consecutive run */
        if (adc_val > thresh_peak)
            thresh_peak = adc_val;

        thresh_consec_count++;

        if (thresh_consec_count >= SOUND_THRESH_CONSECUTIVE)
        {
            /* >>> LOUD SOUND EVENT <<< */
            evt->detected     = 1;
            evt->peak_adc     = thresh_peak;
            evt->timestamp_ms = HAL_GetTick();

            /* Reset and enter cooldown */
            thresh_consec_count = 0;
            thresh_peak         = 0;
            thresh_cooldown     = SOUND_THRESH_COOLDOWN;

#if SOUND_DEBUG_PRINT
            sprintf(dbg_buf,
                    "# LOUD SOUND detected!  peak ADC = %u  (t = %lu ms)\r\n",
                    evt->peak_adc, evt->timestamp_ms);
            Sound_DbgPrint(dbg_buf);
#endif
        }
    }
    else
    {
        /* Below threshold — reset consecutive counter */
        thresh_consec_count = 0;
        thresh_peak         = 0;
    }
#endif /* SOUND_ENABLE_THRESHOLD */
}

/* -------------------------------------------------------------------------
 * Sound_StartRecording
 * -------------------------------------------------------------------------
 * Begins timer-triggered ADC + DMA circular recording.
 * Audio data will be streamed in Sound_RecordingTick().
 * ------------------------------------------------------------------------- */
void Sound_StartRecording(void)
{
#if !SOUND_ENABLE_RECORDING
    return;
#else
    if (rec_state == SOUND_REC_RUNNING)
        return;  /* already recording */

    /* Clear buffer and flags */
    memset(dma_audio_buf, 0, sizeof(dma_audio_buf));
    dma_half_ready  = 0;
    dma_full_ready  = 0;
    rec_samples_sent = 0;
    rec_start_tick   = HAL_GetTick();
    rec_state        = SOUND_REC_RUNNING;

    /* Configure channel for DMA mode (same channel 14) */
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel      = ADC_CHANNEL_14;
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_12CYCLES_5;  /* faster for 8 kHz */
    sConfig.SingleDiff   = ADC_SINGLE_ENDED;
    sConfig.OffsetNumber = ADC_OFFSET_NONE;
    sConfig.Offset       = 0;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    /* Start ADC with DMA in circular mode.
     * TIM3 TRGO triggers each conversion at the desired sample rate. */
    HAL_ADC_Start_DMA(&hadc1,
                       (uint32_t *)dma_audio_buf,
                       2 * SOUND_DMA_HALF_BUF_SIZE);

    /* Start TIM3 — its update event is routed to ADC TRGO */
    HAL_TIM_Base_Start(&htim3);

#if SOUND_DEBUG_PRINT
    Sound_DbgPrint("# Recording STARTED\r\n");
#endif
#endif /* SOUND_ENABLE_RECORDING */
}

/* -------------------------------------------------------------------------
 * Sound_StopRecording
 * ------------------------------------------------------------------------- */
void Sound_StopRecording(void)
{
#if SOUND_ENABLE_RECORDING
    HAL_TIM_Base_Stop(&htim3);
    HAL_ADC_Stop_DMA(&hadc1);
    dma_half_ready = 0;
    dma_full_ready = 0;
    rec_state = SOUND_REC_IDLE;

#if SOUND_DEBUG_PRINT
    sprintf(dbg_buf, "# Recording STOPPED  (%lu samples sent)\r\n",
            rec_samples_sent);
    Sound_DbgPrint(dbg_buf);
#endif
#endif
}

/* -------------------------------------------------------------------------
 * Sound_GetRecState
 * ------------------------------------------------------------------------- */
Sound_RecState_t Sound_GetRecState(void)
{
    return rec_state;
}

/* -------------------------------------------------------------------------
 * Sound_RecordingTick
 * -------------------------------------------------------------------------
 * Called from the main loop.  Checks DMA flags and streams completed
 * half-buffers.  Also enforces maximum recording duration.
 * ------------------------------------------------------------------------- */
void Sound_RecordingTick(void)
{
#if !SOUND_ENABLE_RECORDING
    return;
#else
    if (rec_state != SOUND_REC_RUNNING)
        return;

    /* --- Check maximum duration --- */
    uint32_t elapsed_ms = HAL_GetTick() - rec_start_tick;
    if (elapsed_ms >= (uint32_t)SOUND_MAX_RECORD_SECS * 1000U)
    {
        Sound_StopRecording();
        rec_state = SOUND_REC_DONE;
#if SOUND_DEBUG_PRINT
        Sound_DbgPrint("# Recording reached max duration — DONE\r\n");
#endif
        return;
    }

    /* --- Stream whichever half-buffer is ready --- */
    if (dma_half_ready)
    {
        dma_half_ready = 0;
        /* First half of the buffer is ready (indices 0 .. HALF-1) */
        Sound_StreamHalf(&dma_audio_buf[0], SOUND_DMA_HALF_BUF_SIZE);
    }
    if (dma_full_ready)
    {
        dma_full_ready = 0;
        /* Second half of the buffer is ready (indices HALF .. 2×HALF-1) */
        Sound_StreamHalf(&dma_audio_buf[SOUND_DMA_HALF_BUF_SIZE],
                         SOUND_DMA_HALF_BUF_SIZE);
    }
#endif /* SOUND_ENABLE_RECORDING */
}

/* -------------------------------------------------------------------------
 * Sound_GetRawBuffer
 * -------------------------------------------------------------------------
 * Returns pointer to the DMA buffer for debugger / memory-view inspection.
 * ------------------------------------------------------------------------- */
const uint16_t *Sound_GetRawBuffer(void)
{
    return dma_audio_buf;
}

/*============================================================================
 *  DMA CALLBACKS  (called from DMA ISR via HAL)
 *============================================================================
 * HAL_ADC_ConvHalfCpltCallback → first half of circular buffer filled
 * HAL_ADC_ConvCpltCallback     → second half filled  (buffer wraps)
 *
 * We just set flags here; actual streaming is done in the main loop
 * via Sound_RecordingTick() to avoid blocking in ISR context.
 *============================================================================*/

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
        dma_half_ready = 1;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1)
        dma_full_ready = 1;
}

/*============================================================================
 *  PRIVATE HELPERS
 *============================================================================*/

/* -------------------------------------------------------------------------
 * ADC1_Init  — configure ADC1 for single-ended 12-bit conversions on CH14.
 *
 * In polling mode (threshold), software-triggered.
 * In DMA mode (recording), externally triggered by TIM3 TRGO.
 *
 * We initialise in software-trigger mode here; Sound_StartRecording()
 * reconfigures the trigger to TIM3 before starting DMA.
 * ------------------------------------------------------------------------- */
static void ADC1_Init(void)
{
    __HAL_RCC_ADC_CLK_ENABLE();                    /* Enable ADC clock     */

    hadc1.Instance                   = ADC1;
    hadc1.Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV1;  /* 4 MHz */
    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;     /* 1 channel  */
    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait      = DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;              /* single shot for polling */
    hadc1.Init.NbrOfConversion       = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.NbrOfDiscConversion   = 1;

    /*
     * ExternalTrigConv & ExternalTrigConvEdge:
     *   For polling (threshold) we use SOFTWARE_START.
     *   For recording, Sound_StartRecording() will call HAL_ADC_Stop then
     *   reconfigure and restart with TIM3 trigger.
     *
     *   TIM3 TRGO → ADC external trigger on STM32L4:
     *     ADC_EXTERNALTRIG_T3_TRGO
     *
     *   We set it up for TIM3 from the start; in polling mode we simply
     *   call HAL_ADC_Start() which issues a software trigger anyway.
     */
    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T3_TRGO;
    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;

    hadc1.Init.DMAContinuousRequests = ENABLE;   /* keep DMA running circularly */
    hadc1.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc1.Init.OversamplingMode      = DISABLE;

    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        /* If ADC init fails, blink LED rapidly as error indicator */
        while (1) { }
    }

    /* Run the internal ADC calibration (single-ended mode) */
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
}

/* -------------------------------------------------------------------------
 * TIM3_Init  — configure TIM3 to fire TRGO (update event) at the desired
 *              audio sample rate.
 *
 *   Timer clock  = APB1 timer clock = SYSCLK = 4 MHz  (prescaler 1)
 *   Desired freq = SOUND_SAMPLE_RATE_HZ
 *   ARR          = (4 000 000 / SOUND_SAMPLE_RATE_HZ) − 1
 *
 *   Example: 8 kHz → ARR = 499     → TIM3 overflows every 125 µs
 *            4 kHz → ARR = 999     → TIM3 overflows every 250 µs
 * ------------------------------------------------------------------------- */
static void TIM3_Init(void)
{
    __HAL_RCC_TIM3_CLK_ENABLE();

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;   /* No prescaling — full 4 MHz      */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = (4000000U / SOUND_SAMPLE_RATE_HZ) - 1;
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    {
        while (1) { }
    }

    /* Configure TIM3 TRGO = Update event → triggers ADC */
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    sMasterConfig.MasterOutputTrigger  = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode      = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);
}

/* -------------------------------------------------------------------------
 * DMA1_Init  — configure DMA1 Channel 1 for ADC1.
 *
 * On STM32L4, ADC1 can use DMA1 Channel 1, Request 0.
 * The HAL links the DMA handle to the ADC handle internally.
 * We need to enable the DMA IRQ so the half-transfer and transfer-complete
 * callbacks fire.
 * ------------------------------------------------------------------------- */
static void DMA1_Init(void)
{
    __HAL_RCC_DMA1_CLK_ENABLE();

    hdma_adc1.Instance                 = DMA1_Channel1;
    hdma_adc1.Init.Request             = DMA_REQUEST_0;      /* ADC1 */
    hdma_adc1.Init.Direction           = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc           = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc              = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment    = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode                = DMA_CIRCULAR;
    hdma_adc1.Init.Priority            = DMA_PRIORITY_HIGH;

    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
        while (1) { }
    }

    /* Link DMA handle to ADC handle */
    __HAL_LINKDMA(&hadc1, DMA_Handle, hdma_adc1);

    /* Enable DMA1 Channel 1 interrupt in NVIC */
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

/* -------------------------------------------------------------------------
 * Sound_StreamHalf
 * -------------------------------------------------------------------------
 * Transmit one half-buffer of 16-bit ADC samples.
 *
 * Protocol over UART (raw binary):
 *   [0xAA][0x55]             — sync header
 *   [len >> 8][len & 0xFF]   — number of samples (big-endian uint16)
 *   [sample_0 lo][sample_0 hi] ... — samples as little-endian uint16
 *
 * If SOUND_STREAM_OVER_UART == 0 the data stays in RAM only (debug mode).
 * If SOUND_SEND_OVER_WIFI  == 1 a stub sends data via WiFi instead.
 * ------------------------------------------------------------------------- */
static void Sound_StreamHalf(const uint16_t *buf, uint16_t len)
{
    rec_samples_sent += len;

#if SOUND_STREAM_OVER_UART
    if (pUart == NULL) return;

    /* --- Send sync header + length --- */
    uint8_t header[4];
    header[0] = (uint8_t)(SOUND_UART_FRAME_HEADER >> 8);   /* 0xAA */
    header[1] = (uint8_t)(SOUND_UART_FRAME_HEADER & 0xFF); /* 0x55 */
    header[2] = (uint8_t)(len >> 8);
    header[3] = (uint8_t)(len & 0xFF);
    HAL_UART_Transmit(pUart, header, 4, HAL_MAX_DELAY);

    /* --- Send sample data (raw binary, little-endian) ---
     * Each sample is 2 bytes (uint16_t).  We send the whole block. */
    HAL_UART_Transmit(pUart, (uint8_t *)buf, len * sizeof(uint16_t),
                      HAL_MAX_DELAY);
#endif /* SOUND_STREAM_OVER_UART */

#if SOUND_SEND_OVER_WIFI
    /*
     * TODO: WiFi transmission stub.
     *
     * To send over WiFi using the ISM43362 module on B-L4S5I-IOT01:
     *   1. Initialise the WiFi module (BSP_WIFI_Init / es_wifi driver).
     *   2. Connect to an access point.
     *   3. Open a TCP or UDP socket to a server on your laptop.
     *   4. Send the audio chunk:
     *        WIFI_SendData(socket, (uint8_t *)buf, len * 2);
     *
     * The ISM43362 driver in Drivers/BSP/Components/es_wifi/ provides
     * the low-level SPI transport.  A higher-level API is typically
     * provided by the X-CUBE-WIFI BSP or by wrapping es_wifi calls.
     *
     * For now this is a placeholder.  Enable SOUND_SEND_OVER_WIFI
     * once your WiFi stack is ready.
     */
    (void)buf;
    (void)len;
#endif /* SOUND_SEND_OVER_WIFI */
}

#if SOUND_DEBUG_PRINT
/* -------------------------------------------------------------------------
 * Sound_DbgPrint  — send a debug string over UART
 * All debug lines are prefixed with '#' so a receiver can filter them.
 * ------------------------------------------------------------------------- */
static void Sound_DbgPrint(const char *msg)
{
    if (pUart == NULL) return;
    HAL_UART_Transmit(pUart, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
}
#endif

/*============================================================================
 *  IRQ HANDLERS
 *============================================================================
 * The DMA1 Channel 1 interrupt must be routed to HAL's generic handler
 * so that the half-transfer / transfer-complete callbacks fire.
 *
 * NOTE: If you already have a DMA1_Channel1_IRQHandler elsewhere (e.g.
 *       in stm32l4xx_it.c), move or merge this handler there.
 *       Having two definitions of the same IRQ handler will cause a
 *       linker error — in that case, just add the HAL_DMA_IRQHandler()
 *       call inside the existing function.
 *============================================================================*/
void DMA1_Channel1_IRQHandler(void)
{
    HAL_DMA_IRQHandler(&hdma_adc1);
}
