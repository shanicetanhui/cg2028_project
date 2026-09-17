/******************************************************************************
 * @file           : main.c
 * @brief          : CG2028 Assignment - ElderCare Wearable Safety Companion
 * @author         : Hou Linxin
 * (c) CG2028 Teaching Team
 ******************************************************************************/

/*--------------------------- Includes ---------------------------------------*/
#include "main.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_accelero.h"
#include "../../Drivers/BSP/B-L4S5I-IOT01/stm32l4s5i_iot01_gyro.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

/*--------------------------- Configuration ----------------------------------*/
#define EWMA_ALPHA_ACCEL_PERCENT   25
#define EWMA_ALPHA_GYRO_PERCENT    25
#define NORMAL_LED_DELAY_MS       1000
#define FALL_LED_DELAY_MS          150
#define SAMPLE_PERIOD_MS			50

/* Initial values are deliberately conservative; to be tuned from UART logs. */
#define FREEFALL_THRESHOLD          4.5f
#define FREEFALL_MIN_SAMPLES		2
#define CATCH_THRESHOLD 			7.0f
#define IMPACT_THRESHOLD           11.0f
#define ANGULAR_THRESHOLD          80.0f
#define REBOUND_ACCEL_THRESHOLD 	4.0f
#define REBOUND_DELTA_THRESHOLD 	2.5f
#define IMPACT_WINDOW_MS	      1500U
#define RECOVERY_TIME_MS          3000U

#define FILTER_WARMUP_SAMPLES 20U

static void UART1_Init(void);
static void UART_Send(const char *text);

typedef enum
{
    STATE_NORMAL,
    STATE_CANDIDATE,
    STATE_ALARM
} FallState;

static const char *FallState_ToString(FallState state)
{
    switch (state)
    {
        case STATE_NORMAL:    return "NORMAL";
        case STATE_CANDIDATE: return "CANDIDATE";
        case STATE_ALARM:     return "ALARM";
        default:              return "UNKNOWN";
    }
}

/* Replace hooks with buzzer/OLED driver calls. */
static void Buzzer_Set(uint8_t on) { (void)on; }
static void OLED_ShowState(const char *state) { (void)state; }

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
//int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;

int main(void)
{
    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_ACCELERO_Init();
    BSP_GYRO_Init();
    BSP_LED_Off(LED2);

    /* Previous EWMA outputs. The first test/application sample starts from 0. */
    int accel_ewma_asm[3] = {0, 0, 0};
    int gyro_ewma_asm[3]  = {0, 0, 0};

    /* Reference C states are kept separately for assembly verification. */
    int accel_ewma_c[3] = {0, 0, 0};
    int gyro_ewma_c[3]  = {0, 0, 0};

    unsigned long sample_number = 0;
    FallState fall_state = STATE_NORMAL;
    uint32_t candidate_start = 0;
    uint32_t recovery_start = 0;
    uint8_t impact_seen = 0;
    uint8_t angular_seen = 0;
    uint32_t last_sample = HAL_GetTick();
    uint32_t last_normal_blink = last_sample;
    static uint8_t freefall_count = 0;
    float peak_accel_candidate = 0.0f;

    while (1)
    {
        int16_t accel_raw_i16[3] = {0, 0, 0};
        float gyro_raw_float[3] = {0.0f, 0.0f, 0.0f};
        int gyro_raw_int[3] = {0, 0, 0};

        BSP_ACCELERO_AccGetXYZ(accel_raw_i16);
        BSP_GYRO_GetXYZ(gyro_raw_float);

        /* The supplied BSP reports gyroscope readings as floating-point raw
         * values. Convert them to signed integers before passing them to the
         * integer assembly routine. */
        for (int axis = 0; axis < 3; axis++)
        {
            gyro_raw_int[axis] = (int)gyro_raw_float[axis];

            accel_ewma_asm[axis] = ewma_filter(
                (int)accel_raw_i16[axis],
                accel_ewma_asm[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_asm[axis] = ewma_filter(
                gyro_raw_int[axis],
                gyro_ewma_asm[axis],
                EWMA_ALPHA_GYRO_PERCENT);

            accel_ewma_c[axis] = ewma_filter_C(
                (int)accel_raw_i16[axis],
                accel_ewma_c[axis],
                EWMA_ALPHA_ACCEL_PERCENT);

            gyro_ewma_c[axis] = ewma_filter_C(
                gyro_raw_int[axis],
                gyro_ewma_c[axis],
                EWMA_ALPHA_GYRO_PERCENT);
        }

        /* Accelerometer filtered readings are in meters per second squared. */
        float accel_mps2[3] = {
            accel_ewma_asm[0] * (9.80665f / 1000.0f),
            accel_ewma_asm[1] * (9.80665f / 1000.0f),
            accel_ewma_asm[2] * (9.80665f / 1000.0f)
        };

        /* Gyroscope filtered readings are in degrees per second. */
        float gyro_dps[3] = {
            gyro_ewma_asm[0] / 1000.0f,
            gyro_ewma_asm[1] / 1000.0f,
            gyro_ewma_asm[2] / 1000.0f
        };

        char raw_values_buffer[320];
        int len = snprintf(raw_values_buffer, sizeof(raw_values_buffer),
                 "Sample %lu\r\n"
                 "Accel EWMA ASM [m/s^2]: X=%8.3f Y=%8.3f Z=%8.3f\r\n"
                 "Gyro  EWMA ASM [dps]  : X=%8.3f Y=%8.3f Z=%8.3f\r\n",
                 sample_number,
                 accel_mps2[0], accel_mps2[1], accel_mps2[2],
                 gyro_dps[0], gyro_dps[1], gyro_dps[2]);
//        UART_Send(raw_values_buffer);

        if (sample_number < FILTER_WARMUP_SAMPLES)
        {
            fall_state = STATE_NORMAL;
            impact_seen = 0;
            angular_seen = 0;
            Buzzer_Set(0);
            BSP_LED_Off(LED2);
            OLED_ShowState("STARTING");

            HAL_Delay(SAMPLE_PERIOD_MS);
            sample_number++;
            continue;
        }

        /* Optional debugging check. This confirms that the assembly routine
         * matches the reference C routine for the current samples. */
        if ((accel_ewma_asm[0] != accel_ewma_c[0]) ||
            (accel_ewma_asm[1] != accel_ewma_c[1]) ||
            (accel_ewma_asm[2] != accel_ewma_c[2]) ||
            (gyro_ewma_asm[0] != gyro_ewma_c[0]) ||
            (gyro_ewma_asm[1] != gyro_ewma_c[1]) ||
            (gyro_ewma_asm[2] != gyro_ewma_c[2]))
        {
            UART_Send("WARNING: Assembly and C EWMA outputs do not match.\r\n");
        }

        /**************** Elderly wearable state logic starts here************************
         * Compulsory requirements:
         * 1. Use filtered accelerometer AND gyroscope readings.
         * 2. Distinguish normal activity, near-fall movements, and a real fall.
         * 3. Use a slow LED blink for normal operation and a fast blink after
         *    a fall is detected.
         *********************************************************************/

        float accel_norm = sqrtf(accel_mps2[0] * accel_mps2[0] +
        						accel_mps2[1] * accel_mps2[1] +
								accel_mps2[2] * accel_mps2[2]);

        float gyro_norm = sqrtf(gyro_dps[0] * gyro_dps[0] +
								gyro_dps[1] * gyro_dps[1] +
								gyro_dps[2] * gyro_dps[2]);


        char norm_values_buffer[120];
        snprintf(norm_values_buffer, sizeof(norm_values_buffer),
                 "Accel norm = %.2f m/s^2, Gyro norm = %.2f dps\r\n",
                 accel_norm, gyro_norm);
//        UART_Send(norm_values_buffer);


        uint32_t now = HAL_GetTick();

        if (fall_state == STATE_NORMAL) {
            if (accel_norm < FREEFALL_THRESHOLD) {
                freefall_count++;
                if (freefall_count >= FREEFALL_MIN_SAMPLES) {
                    fall_state = STATE_CANDIDATE;
                    candidate_start = now;
                    impact_seen = angular_seen = 0;
                    peak_accel_candidate = accel_norm;
                    freefall_count = 0;
                    OLED_ShowState("POSSIBLE FALL");
                }
            } else {
                freefall_count = 0;   // reset if it doesn't stay low
            }
        }

        if (fall_state == STATE_CANDIDATE) {
            if (accel_norm > peak_accel_candidate) {
                peak_accel_candidate = accel_norm;
            }

            if ((now - candidate_start) > IMPACT_WINDOW_MS) {
                fall_state = STATE_NORMAL;
                OLED_ShowState("NORMAL");
            } else {
                if (accel_norm > CATCH_THRESHOLD)  impact_seen  = 1;
                if (gyro_norm  > ANGULAR_THRESHOLD) angular_seen = 1;

                if (impact_seen || angular_seen) {
					fall_state = STATE_ALARM;
					recovery_start = 0;
					Buzzer_Set(1);
					BSP_LED_On(LED2);
					OLED_ShowState("FALL DETECTED");
					UART_Send("ALARM: fall confirmed\r\n");
				}
            }
        }

        if (fall_state == STATE_ALARM) {
        	/* LED remains on and buzzer remain active until stable recovery. */
        	BSP_LED_On(LED2);
        	if (accel_norm > 7.5f && accel_norm < 12.5f && gyro_norm < 35.0f) {
        		if (!recovery_start) recovery_start = now;
        		if ((now - recovery_start) >= RECOVERY_TIME_MS) {
        			fall_state = STATE_NORMAL;
        			recovery_start = 0;
        			Buzzer_Set(0);
        			BSP_LED_Off(LED2);
        			OLED_ShowState("NORMAL");
        			UART_Send("INFO: recovery detected\r\n");
        		}
        	} else {
        		recovery_start = 0;
        	}
        }

        if (fall_state == STATE_NORMAL) {
        	/* Normal operation: 1 second heartbeat blink. */
        	if ((now - last_normal_blink) >= NORMAL_LED_DELAY_MS) {
        		BSP_LED_Toggle(LED2);
        		last_normal_blink = now;
        	}
        }

        if (len > 0 && len < (int)sizeof(raw_values_buffer))
        {
        	snprintf(raw_values_buffer + len, sizeof(raw_values_buffer) - len,
        	                     "State: %s, Peak accel (candidate): %.2f m/s^2\r\n",
								 FallState_ToString(fall_state), peak_accel_candidate);
        }
        UART_Send(raw_values_buffer);
        UART_Send(norm_values_buffer);

        if ((HAL_GetTick() - last_sample) < SAMPLE_PERIOD_MS) {
        	HAL_Delay(SAMPLE_PERIOD_MS - (HAL_GetTick() - last_sample));
        }
        last_sample = HAL_GetTick();

        sample_number++;
    }
}

int ewma_filter_C(int new_data, int old_output, int alpha_percent)
{
    /* Reference implementation for verification only. The assembly routine
     * must be used in the actual sensor-processing and detection pipeline. */
    int numerator = alpha_percent * new_data
                  + (100 - alpha_percent) * old_output;
    return numerator / 100;
}

static void UART_Send(const char *text)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)text, strlen(text), HAL_MAX_DELAY);
}

static void UART1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    GPIO_InitStruct.Pin = GPIO_PIN_7 | GPIO_PIN_6;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
        while (1) { }
    }
}

/* Do not modify these lines. They suppress UART-related warnings. */
int _write(int file, char *ptr, int len)
{
    (void)file;
    (void)ptr;
    return len;
}
int _read(int file, char *ptr, int len) { (void)file; (void)ptr; (void)len; return 0; }
int _fstat(int file, struct stat *st) { (void)file; (void)st; return 0; }
int _lseek(int file, int ptr, int dir) { (void)file; (void)ptr; (void)dir; return 0; }
int _isatty(int file) { (void)file; return 1; }
int _close(int file) { (void)file; return -1; }
int _getpid(void) { return 1; }
int _kill(int pid, int sig) { (void)pid; (void)sig; return -1; }
