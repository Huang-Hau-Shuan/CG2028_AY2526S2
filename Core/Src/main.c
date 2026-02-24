  /******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * (c) CG2028 Teaching Team
  ******************************************************************************/


/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_tsensor.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include "stdio.h"
#include "string.h"
#include <sys/stat.h>

static void UART1_Init(void);

extern void initialise_monitor_handles(void);	// for semi-hosting support (printf). Will not be required if transmitting via UART

extern int mov_avg(int N, int* accel_buff); // asm implementation

int mov_avg_C(int N, int* accel_buff); // Reference C implementation

UART_HandleTypeDef huart1;


int main(void)
{
	const int N=4;

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* UART initialization  */
	UART1_Init();

	/* Peripheral initializations using BSP functions */
	BSP_LED_Init(LED2);
	BSP_ACCELERO_Init();
	BSP_GYRO_Init();

	/*Set the initial LED state to off*/
	BSP_LED_Off(LED2);

	int accel_buff_x[4]={0};
	int accel_buff_y[4]={0};
	int accel_buff_z[4]={0};
	int i=0;
	int delay_ms=200; // 200 ms loop → 5 Hz sampling for fall detection

	/*--- Fall-detection state machine (Apple-Watch-inspired) ---
	 * Phase 0  IDLE:            monitor for free-fall or big spike
	 * Phase 1  FREEFALL_PHASE:  free-fall seen → wait for impact + rotation
	 * Phase 2  FALL_CONFIRMED:  fast-blink, print alert, cool down
	 *-----------------------------------------------------------*/
	int fall_state     = 0;   // current phase
	int state_timer    = 0;   // iterations left in the current window
	int fall_cooldown  = 0;   // iterations of fast-blink remaining

	// LED pacing (decoupled from loop delay so blink rate is independent)
	int led_counter    = 0;
	const int LED_SLOW = 5;   // toggle every 5×200 ms = 1 s  (≈ 0.5 Hz blink)
	const int LED_FAST = 1;   // toggle every 1×200 ms = 200 ms during fall cooldown

	while (1)
	{

		// --- LED blink ---
		led_counter++;
		if(fall_state == 2)
		{
			if(led_counter >= LED_FAST) { BSP_LED_Toggle(LED2); led_counter = 0; }
		}
		else
		{
			if(led_counter >= LED_SLOW) { BSP_LED_Toggle(LED2); led_counter = 0; }
		}

		int16_t accel_data_i16[3] = { 0 };			// array to store the x, y and z readings of accelerometer
		/********Function call to read accelerometer values*********/
		BSP_ACCELERO_AccGetXYZ(accel_data_i16);

		//Copy the values over to a circular style buffer
		accel_buff_x[i%4]=accel_data_i16[0]; //acceleration along X-Axis
		accel_buff_y[i%4]=accel_data_i16[1]; //acceleration along Y-Axis
		accel_buff_z[i%4]=accel_data_i16[2]; //acceleration along Z-Axis


		// ********* Read gyroscope values *********/
		float gyro_data[3]={0.0};
		float* ptr_gyro=gyro_data;
		BSP_GYRO_GetXYZ(ptr_gyro);

		//The output of gyro has been made to display in dps(degree per second)
		float gyro_velocity[3]={0.0};
		gyro_velocity[0]=(gyro_data[0]*9.8/(1000));
		gyro_velocity[1]=(gyro_data[1]*9.8/(1000));
		gyro_velocity[2]=(gyro_data[2]*9.8/(1000));


		//Preprocessing the filtered outputs  The same needs to be done for the output from the C program as well
		float accel_filt_asm[3]={0}; // final value of filtered acceleration values

		accel_filt_asm[0]= (float)mov_avg(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_asm[1]= (float)mov_avg(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_asm[2]= (float)mov_avg(N,accel_buff_z) * (9.8/1000.0f);


		//Preprocessing the filtered outputs  The same needs to be done for the output from the assembly program as well
		float accel_filt_c[3]={0};

		accel_filt_c[0]=(float)mov_avg_C(N,accel_buff_x) * (9.8/1000.0f);
		accel_filt_c[1]=(float)mov_avg_C(N,accel_buff_y) * (9.8/1000.0f);
		accel_filt_c[2]=(float)mov_avg_C(N,accel_buff_z) * (9.8/1000.0f);

		/***************************UART transmission*******************************************/
		char buffer[150]; // Create a buffer large enough to hold the text

		/******Transmitting results of C execution over UART*********/
		if(i>=3)
		{
			// 1. First printf() Equivalent
			sprintf(buffer, "Results of C execution for filtered accelerometer readings:\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// 2. Second printf() (with Floats) Equivalent
			// Note: Requires -u _printf_float to be enabled in Linker settings
			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
					accel_filt_c[0], accel_filt_c[1], accel_filt_c[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			/******Transmitting results of asm execution over UART*********/

			// 1. First printf() Equivalent
			sprintf(buffer, "Results of assembly execution for filtered accelerometer readings:\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// 2. Second printf() (with Floats) Equivalent
			// Note: Requires -u _printf_float to be enabled in Linker settings
			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n",
					accel_filt_asm[0], accel_filt_asm[1], accel_filt_asm[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			/******Transmitting Gyroscope readings over UART*********/

			// 1. First printf() Equivalent
			sprintf(buffer, "Gyroscope sensor readings:\r\n");
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

			// 2. Second printf() (with Floats) Equivalent
			// Note: Requires -u _printf_float to be enabled in Linker settings
			sprintf(buffer, "Averaged X : %f; Averaged Y : %f; Averaged Z : %f;\r\n\n",
					gyro_velocity[0], gyro_velocity[1], gyro_velocity[2]);
			HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
		}

		HAL_Delay(delay_ms);	// 1 second delay

		i++;

		// ********* Fall detection (Apple-Watch-inspired state machine) *********/
		if(i >= 3)
		{
			/* ---- Magnitude calculations ---- */

			// 1. Filtered acceleration magnitude² (smoothed, in m/s²)
			float accel_mag_sq = accel_filt_asm[0] * accel_filt_asm[0]
			                   + accel_filt_asm[1] * accel_filt_asm[1]
			                   + accel_filt_asm[2] * accel_filt_asm[2];

			// 2. Raw (unfiltered, current sample) magnitude² — catches brief spikes
			float raw_x = (float)accel_data_i16[0] * (9.8f / 1000.0f);
			float raw_y = (float)accel_data_i16[1] * (9.8f / 1000.0f);
			float raw_z = (float)accel_data_i16[2] * (9.8f / 1000.0f);
			float raw_mag_sq = raw_x*raw_x + raw_y*raw_y + raw_z*raw_z;

			// 3. Gyroscope magnitude² (in the code's converted units)
			float gyro_mag_sq = gyro_velocity[0] * gyro_velocity[0]
			                  + gyro_velocity[1] * gyro_velocity[1]
			                  + gyro_velocity[2] * gyro_velocity[2];

			/* ---- Thresholds (all squared, no sqrt needed) ----
			 *
			 * Reference: 1 g rest = 9.8 m/s²  →  mag_sq ≈ 96
			 *
			 * FREE-FALL : accel drops well below 1 g
			 *   raw  < 0.4 g  → raw_mag_sq  < 15     ← instant sample
			 *   filt < 0.6 g  → accel_mag_sq < 35    ← smoothed
			 *
			 * IMPACT    : accel spikes above 1 g
			 *   raw  > 2.0 g  → raw_mag_sq  > 385
			 *   filt > 1.8 g  → accel_mag_sq > 310
			 *
			 * DIRECT HIT: very hard impact even without clear free-fall
			 *   raw  > 3.0 g  → raw_mag_sq  > 865
			 *
			 * GYROSCOPE : rapid orientation change (≥ ~30 dps total)
			 *   gyro_mag_sq > 80 000
			 *   (gyro units from code:  mdps × 9.8 / 1000)
			 */
			const float FF_RAW_SQ      = 15.0f;
			const float FF_FILT_SQ     = 35.0f;
			const float IMPACT_RAW_SQ  = 385.0f;
			const float IMPACT_FILT_SQ = 310.0f;
			const float DIRECT_HIT_SQ  = 865.0f;
			const float GYRO_SQ        = 80000.0f;

			switch(fall_state)
			{
			/* ---- Phase 0: IDLE — watch for anomaly ---- */
			case 0:
				// a) Free-fall detected in raw OR filtered
				if(raw_mag_sq < FF_RAW_SQ || accel_mag_sq < FF_FILT_SQ)
				{
					fall_state  = 1;
					state_timer = 15; // 15 × 200 ms = 3 s window for impact
				}
				// b) Extremely hard impact without observable free-fall
				else if(raw_mag_sq > DIRECT_HIT_SQ && gyro_mag_sq > GYRO_SQ)
				{
					fall_state   = 2;
					fall_cooldown = 25; // 25 × 200 ms = 5 s fast-blink
					sprintf(buffer, "*** FALL DETECTED (direct impact) ***\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
				}
				break;

			/* ---- Phase 1: FREE-FALL seen — look for impact + rotation ---- */
			case 1:
				if((raw_mag_sq > IMPACT_RAW_SQ || accel_mag_sq > IMPACT_FILT_SQ)
					&& gyro_mag_sq > GYRO_SQ)
				{
					// Impact with rotation → fall confirmed
					fall_state    = 2;
					fall_cooldown = 25;
					state_timer   = 0;
					sprintf(buffer, "*** FALL DETECTED (freefall+impact) ***\r\n");
					HAL_UART_Transmit(&huart1, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
				}
				else if(--state_timer <= 0)
				{
					fall_state = 0; // window expired — false alarm
				}
				break;

			/* ---- Phase 2: FALL CONFIRMED — fast-blink cooldown ---- */
			case 2:
				if(--fall_cooldown <= 0)
				{
					fall_state   = 0;
					fall_cooldown = 0;
				}
				break;
			}
		}

	}


}



int mov_avg_C(int N, int* accel_buff)
{ 	// The implementation below is inefficient and meant only for verifying your results.
	int result=0;
	for(int i=0; i<N;i++)
	{
		result+=accel_buff[i];
	}

	result=result/4;

	return result;
}

static void UART1_Init(void)
{
        /* Pin configuration for UART. BSP_COM_Init() can do this automatically */
        __HAL_RCC_GPIOB_CLK_ENABLE();
         __HAL_RCC_USART1_CLK_ENABLE();

        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
        GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* Configuring UART1 */
        huart1.Instance = USART1;
        huart1.Init.BaudRate = 115200;
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        huart1.Init.StopBits = UART_STOPBITS_1;
        huart1.Init.Parity = UART_PARITY_NONE;
        huart1.Init.Mode = UART_MODE_TX_RX;
        huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
        huart1.Init.OverSampling = UART_OVERSAMPLING_16;
        huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
        huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
        if (HAL_UART_Init(&huart1) != HAL_OK)
        {
          while(1);
        }

}


// Do not modify these lines of code. They are written to supress UART related warnings
int _read(int file, char *ptr, int len) { return 0; }
int _fstat(int file, struct stat *st) { return 0; }
int _lseek(int file, int ptr, int dir) { return 0; }
int _isatty(int file) { return 1; }
int _close(int file) { return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { return -1; }
