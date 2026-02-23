/******************************************************************************
 * @file    sound_sensor.h
 * @brief   Grove Sound Sensor driver for fall-detection system
 *
 * Hardware: Grove - Sound Sensor v1.6 connected to Arduino-header A0
 *           (pin PC5 on B-L4S5I-IOT01, ADC1 channel 14)
 *
 * Two operating modes:
 *   1. THRESHOLD detection  – polled in the main loop, flags loud events
 *      (thump / scream) to complement the on-board digital microphones.
 *   2. AUDIO RECORDING      – timer-triggered ADC via DMA fills a circular
 *      buffer; data is streamed over UART (or saved locally for debug).
 *
 * All tuneable parameters are gathered in the "Configuration" section below
 * so you can adjust behaviour without touching the rest of the source.
 ******************************************************************************/
#ifndef SOUND_SENSOR_H
#define SOUND_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>

/*============================================================================
 *  CONFIGURATION — edit these #defines to tune behaviour / debug
 *============================================================================*/

/* ---- Feature switches (set to 1 to enable, 0 to disable) ---- */

/**
 * @brief Enable threshold-based loud-sound detection.
 *        When enabled, the main loop polls the ADC and checks if the
 *        sound level exceeds SOUND_THRESH_HIGH.
 */
#define SOUND_ENABLE_THRESHOLD      1

/**
 * @brief Enable audio recording (timer-triggered ADC + DMA).
 *        When enabled, Sound_StartRecording() / Sound_StopRecording()
 *        become functional.
 */
#define SOUND_ENABLE_RECORDING      1

/**
 * @brief When set to 1, recorded audio is streamed over UART as raw
 *        binary bytes to your PC (you can capture with a serial terminal).
 *        When set to 0, audio is only stored in the internal RAM buffer
 *        (useful for stepping through with a debugger / memory view).
 */
#define SOUND_STREAM_OVER_UART      1

/**
 * @brief When set to 1, recorded audio is sent over WiFi (ISM43362)
 *        to a remote endpoint.  Requires WiFi to be initialised separately.
 *        >>> Set to 0 for local-only / debug builds. <<<
 */
#define SOUND_SEND_OVER_WIFI        0

/**
 * @brief Print human-readable debug messages over UART (sound level,
 *        threshold events, recording status, etc.).
 *        Disable in production to reduce UART traffic.
 */
#define SOUND_DEBUG_PRINT           1

/* ---- Threshold detection parameters ---- */

/**
 * @brief ADC reading above which a "loud sound" event is flagged.
 *        ADC is 12-bit (0-4095).  The Grove Sound Sensor outputs ~1.5-2 V
 *        at rest for a 3.3 V reference; loud sounds push it above 3 V.
 *        Tune this value experimentally:
 *          quiet room  ≈ 200-600
 *          normal talk  ≈ 600-1200
 *          clap / thump ≈ 2000+
 *          shout / yell ≈ 2500+
 */
#define SOUND_THRESH_HIGH           2000

/**
 * @brief Number of consecutive above-threshold ADC polls required before
 *        a loud-sound event is declared.  Prevents single-sample glitches
 *        from causing false positives.
 */
#define SOUND_THRESH_CONSECUTIVE    3

/**
 * @brief Cooldown (in main-loop iterations) after a loud-sound event
 *        before another can be declared.  Avoids retriggering on the
 *        same event.
 */
#define SOUND_THRESH_COOLDOWN       25    /* 25 × 200 ms = 5 s */

/* ---- Audio recording parameters ---- */

/**
 * @brief Sample rate for audio recording in Hz.
 *        8000 Hz is telephone quality – adequate for the Grove sensor.
 *        4000 Hz halves the data rate and still captures voice.
 *        Adjust based on available UART bandwidth / WiFi throughput.
 *
 *        At 115200 baud UART (≈ 11.5 KB/s raw binary):
 *          8000 Hz × 1 byte = 8 KB/s   → fits
 *          8000 Hz × 2 byte = 16 KB/s  → too fast for 115200
 *        So when streaming over UART we transmit 8-bit samples.
 */
#define SOUND_SAMPLE_RATE_HZ        8000

/**
 * @brief Maximum recording duration in seconds.
 *        The DMA buffer is sized for a shorter chunk; the recording
 *        logic loops over chunks up to this total duration.
 *        Set to 120 for ~2 minutes.
 */
#define SOUND_MAX_RECORD_SECS       120

/**
 * @brief Size of each DMA half-buffer in samples.
 *        A double-buffer scheme is used: while one half is being filled
 *        by DMA, the other half is transmitted.
 *        Total DMA buffer = 2 × SOUND_DMA_HALF_BUF_SIZE × sizeof(uint16_t).
 *
 *        With 8 kHz and half-buffer of 2048:
 *          half-buffer fill time = 2048 / 8000 = 0.256 s
 *          total DMA buffer      = 2 × 2048 × 2 = 8192 bytes
 */
#define SOUND_DMA_HALF_BUF_SIZE     2048

/* ---- UART streaming format ---- */

/**
 * @brief Start-of-frame marker sent before each audio chunk so the
 *        receiving script can synchronise.
 */
#define SOUND_UART_FRAME_HEADER     0xAA55

/*============================================================================
 *  DATA TYPES
 *============================================================================*/

/** @brief Possible states of the recording engine */
typedef enum {
    SOUND_REC_IDLE = 0,      /**< Not recording                            */
    SOUND_REC_RUNNING,       /**< DMA is filling, data being streamed      */
    SOUND_REC_DONE           /**< Max duration reached; data fully sent    */
} Sound_RecState_t;

/** @brief Summary of a loud-sound event (for the fall-detection FSM) */
typedef struct {
    uint8_t  detected;       /**< 1 if a loud-sound event is active        */
    uint16_t peak_adc;       /**< Highest ADC value during the event       */
    uint32_t timestamp_ms;   /**< HAL_GetTick() at the moment of detection */
} Sound_ThreshEvent_t;

/*============================================================================
 *  PUBLIC API
 *============================================================================*/

/**
 * @brief  Initialise ADC1 on PC5 (ARD_A0) for the Grove Sound Sensor.
 *         Also sets up TIM3 for timer-triggered recording (if enabled).
 *         Call once during startup, after HAL_Init().
 * @param  huart  Pointer to the UART handle used for streaming / debug.
 *                Pass NULL if you don't want any UART output.
 */
void Sound_Init(UART_HandleTypeDef *huart);

/**
 * @brief  Perform a single blocking ADC conversion and return the raw
 *         12-bit value (0-4095).  Useful for threshold polling in the
 *         main loop.
 * @return ADC reading (0-4095), or 0 on error.
 */
uint16_t Sound_ReadADC(void);

/**
 * @brief  Run the threshold-detection algorithm.  Call this once per
 *         main-loop iteration.  It polls the ADC, applies the
 *         consecutive-count and cooldown logic, and updates the event
 *         structure.
 * @param  evt  Pointer to a Sound_ThreshEvent_t to be filled in.
 *              evt->detected is set to 1 on a new loud-sound event.
 */
void Sound_ThresholdCheck(Sound_ThreshEvent_t *evt);

/**
 * @brief  Start recording audio from the Grove sensor.
 *         Uses TIM3-triggered ADC + DMA in circular mode.
 *         Data is streamed according to the SOUND_STREAM_OVER_UART /
 *         SOUND_SEND_OVER_WIFI configuration flags.
 * @note   Recording runs asynchronously via DMA interrupts.
 *         Call Sound_RecordingTick() periodically (e.g. in the main loop)
 *         to handle streaming and timeout.
 */
void Sound_StartRecording(void);

/**
 * @brief  Stop an ongoing recording immediately.
 */
void Sound_StopRecording(void);

/**
 * @brief  Returns the current recording state.
 */
Sound_RecState_t Sound_GetRecState(void);

/**
 * @brief  Must be called periodically (e.g. every main-loop iteration)
 *         while a recording is in progress.  Handles:
 *           - streaming completed DMA half-buffers over UART / WiFi
 *           - checking if maximum recording duration elapsed
 */
void Sound_RecordingTick(void);

/**
 * @brief  Get read-only pointer to the raw DMA buffer for debugger
 *         inspection.  Length = 2 × SOUND_DMA_HALF_BUF_SIZE.
 */
const uint16_t *Sound_GetRawBuffer(void);

#ifdef __cplusplus
}
#endif

#endif /* SOUND_SENSOR_H */
