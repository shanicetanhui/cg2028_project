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
#include "ssd1306.h"
#include "ssd1306_fonts.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>

/*--------------------------- Configuration ----------------------------------*/
#define EWMA_ALPHA_ACCEL_PERCENT  	25
#define EWMA_ALPHA_GYRO_PERCENT   	25

#define SAMPLE_PERIOD_MS			50

/* LED Blink */
#define NORMAL_LED_DELAY_MS      	1000
#define FALL_LED_DELAY_MS         	150

/* Fall detection*/
#define FREEFALL_THRESHOLD          4.5f
#define FREEFALL_MIN_SAMPLES		2
#define CATCH_THRESHOLD 			7.0f
//#define IMPACT_THRESHOLD            11.0f
#define ANGULAR_THRESHOLD      	    80.0f
//#define REBOUND_ACCEL_THRESHOLD 	4.0f
//#define REBOUND_DELTA_THRESHOLD 	2.5f
#define IMPACT_WINDOW_MS	      	1500U
//#define RECOVERY_BTN_TIMEOUT_MS  	10000U
#define EMERGENCY_BLINK_MS			500U
#define LONG_LIE_TIMEOUT_MS       10000U   // no movement at all -> emergency
#define MOVEMENT_GRACE_TIMEOUT_MS 20000U   // movement seen -> extended grace period
#define MOVEMENT_ACCEL_DELTA         1.0f  // m/s^2 sample-to-sample change = movement
#define MOVEMENT_GYRO_THRESHOLD     15.0f  // dps, well below ANGULAR_THRESHOLD (80)

#define FILTER_WARMUP_SAMPLES 		1U

static void UART1_Init(void);
static void UART_Send(const char *text);
static void I2C1_Init(void);
static void Buzzer_Init(void);
static uint32_t last_buzzer_toggle = 0;
static uint8_t buzzer_on = 0;

typedef enum
{
    STATE_NORMAL,
    STATE_CANDIDATE,
    STATE_ALARM,
	STATE_EMERGENCY
} FallState;

static char *FallState_ToString(FallState state)
{
    switch (state)
    {
        case STATE_NORMAL:    return "NORMAL";
        case STATE_CANDIDATE: return "POTENTIAL";
        case STATE_ALARM:     return "FALL";
        case STATE_EMERGENCY: return "CALL 995";
        default:              return "UNKNOWN";
    }
}

/* Replace hooks with buzzer/OLED driver calls. */
static void Buzzer_Set(uint8_t on)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static volatile uint8_t button_flag = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    static uint32_t last_press_tick = 0;
    uint32_t now = HAL_GetTick();
    if ((now - last_press_tick) > 200) {   // simple 200ms debounce
        button_flag = 1;
        last_press_tick = now;
    }
}

static uint8_t Button_WasPressed(void)
{
	if (button_flag) {
		button_flag = 0;
		return 1;
	}
	return 0;
}

static void OLED_WriteCentered(uint8_t y, const char *str, SSD1306_Font_t font, SSD1306_COLOR color)
{
    size_t len = strlen(str);
    uint16_t text_width = 0;

    if (font.char_width) {
        for (size_t i = 0; i < len; i++) {
            char ch = str[i];
            if (ch < 32 || ch > 126) continue;
            text_width += font.char_width[ch - 32];
        }
    } else {
        text_width = font.width * len;
    }

    uint8_t x = (text_width < SSD1306_WIDTH) ? (SSD1306_WIDTH - text_width) / 2 : 0;
    ssd1306_SetCursor(x, y);
    ssd1306_WriteString((char *)str, font, color);
}

static void OLED_ShowState(FallState state)
{
    ssd1306_Fill(Black);
    const char* state_str = FallState_ToString(state);
    if (state == STATE_ALARM || state == STATE_EMERGENCY) {
    	OLED_WriteCentered(19, state_str, Font_16x26, White);
    } else {
    	OLED_WriteCentered(23, state_str, Font_11x18, White);
    }
    ssd1306_UpdateScreen();
}

extern int ewma_filter(int new_data, int old_output, int alpha_percent);
//int ewma_filter_C(int new_data, int old_output, int alpha_percent);

UART_HandleTypeDef huart1;
I2C_HandleTypeDef hi2c1;

int main(void)
{
    HAL_Init();
    UART1_Init();

    BSP_LED_Init(LED2);
    BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);

    BSP_ACCELERO_Init();
    BSP_GYRO_Init();

    I2C1_Init();
    ssd1306_Init();
    Buzzer_Init();

    /* Testing buzzer:
    while (1)
    {
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
        HAL_Delay(5000);   // ON for 5 seconds

        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
        HAL_Delay(5000);   // OFF for 5 seconds
    } */

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
    uint8_t impact_seen = 0;
    uint8_t angular_seen = 0;
    uint32_t last_sample = HAL_GetTick();
    uint32_t last_normal_blink = last_sample;
    static uint8_t freefall_count = 0;
    float peak_accel_candidate = 0.0f;
    uint32_t alarm_start = 0;
    uint32_t last_alarm_blink = last_sample;
    uint32_t last_emergency_toggle = last_sample;
    uint8_t emergency_oled_on = 0;
    uint8_t movement_detected_in_alarm = 0;
    float prev_accel_norm = 0.0f;

    OLED_ShowState(fall_state);
    UART_Send("INFO: Beginning fall detection\r\n");

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

        if (sample_number < FILTER_WARMUP_SAMPLES)
        {
            fall_state = STATE_NORMAL;
            impact_seen = 0;
            angular_seen = 0;
            Buzzer_Set(0);
            BSP_LED_Off(LED2);

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


        uint32_t now = HAL_GetTick();

        if (fall_state == STATE_NORMAL) {
           	/* Normal operation: 1 second heartbeat blink. */
			if ((now - last_normal_blink) >= NORMAL_LED_DELAY_MS) {
				BSP_LED_Toggle(LED2);
				last_normal_blink = now;
			}

            if (accel_norm < FREEFALL_THRESHOLD) {
                freefall_count++;
                if (freefall_count >= FREEFALL_MIN_SAMPLES) {
                    fall_state = STATE_CANDIDATE;
                    candidate_start = now;
                    impact_seen = angular_seen = 0;
                    peak_accel_candidate = accel_norm;
                    freefall_count = 0;
                    OLED_ShowState(fall_state);
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
                OLED_ShowState(fall_state);
            } else {
                if (accel_norm > CATCH_THRESHOLD)  impact_seen  = 1;
                if (gyro_norm  > ANGULAR_THRESHOLD) angular_seen = 1;

                if (impact_seen || angular_seen) {
					fall_state = STATE_ALARM;
					alarm_start = now;
					last_alarm_blink = now;
					movement_detected_in_alarm = 0;
					Buzzer_Set(1);
					BSP_LED_On(LED2);
					OLED_ShowState(fall_state);
					UART_Send("ALARM: fall confirmed, press blue button when recovered\r\n");
				}
            }
        }

        if (fall_state == STATE_ALARM) {
            /* Fast LED blink while alarm is active. */
            if ((now - last_alarm_blink) >= FALL_LED_DELAY_MS) {
                BSP_LED_Toggle(LED2);
                last_alarm_blink = now;
            }
            /* Buzzer toggling on and off. */
            if ((now - last_buzzer_toggle) >= 300)
            {
                buzzer_on = !buzzer_on;
                Buzzer_Set(buzzer_on);
                last_buzzer_toggle = now;
            }

            /* Track whether any movement has occurred since alarm started. Ignore movement in 1st second. */
            if (!movement_detected_in_alarm &&
                (now - alarm_start) >= 1000U &&
                (fabsf(accel_norm - prev_accel_norm) > MOVEMENT_ACCEL_DELTA ||
                 gyro_norm > MOVEMENT_GYRO_THRESHOLD)) {
                movement_detected_in_alarm = 1;
                UART_Send("INFO: movement detected during alarm, extending grace period to 20s\r\n");
            }

            uint32_t escalation_timeout = movement_detected_in_alarm
                                         ? MOVEMENT_GRACE_TIMEOUT_MS
                                         : LONG_LIE_TIMEOUT_MS;

            if (Button_WasPressed()) {
                fall_state = STATE_NORMAL;
                buzzer_on = 0;
                Buzzer_Set(0);
                BSP_LED_Off(LED2);
                OLED_ShowState(fall_state);
                UART_Send("INFO: recovery confirmed by button press\r\n");
            }
            else if ((now - alarm_start) >= escalation_timeout) {
                fall_state = STATE_EMERGENCY;
                last_emergency_toggle = now;
                emergency_oled_on = 0;
                if (movement_detected_in_alarm) {
                    UART_Send("ALARM: movement seen but no button press, escalating to emergency\r\n");
                } else {
                    UART_Send("ALARM: no movement detected (long lie), escalating to emergency\r\n");
                }
            }
        }

        if (fall_state == STATE_EMERGENCY) {
        	/* Keep LED fast-blinking and buzzer on as before. */
        	if ((now - last_alarm_blink) >= FALL_LED_DELAY_MS) {
				BSP_LED_Toggle(LED2);
				last_alarm_blink = now;
			}

        	Buzzer_Set(1);

        	/* Flash "CALL 995" on OLED. */
        	if ((now - last_emergency_toggle) >= EMERGENCY_BLINK_MS) {
        		emergency_oled_on = !emergency_oled_on;
        		last_emergency_toggle = now;

        		ssd1306_Fill(Black);
        		if (emergency_oled_on) {
        			ssd1306_SetCursor(0, 19);
        			ssd1306_WriteString(FallState_ToString(fall_state), Font_16x26, White);
//        			OLED_ShowState(fall_state);
        		}
        		ssd1306_UpdateScreen();
        	}

        	if (Button_WasPressed()) {
        		fall_state = STATE_NORMAL;
        		buzzer_on = 0;
        		Buzzer_Set(0);
        		BSP_LED_Off(LED2);
        		OLED_ShowState(fall_state);
        		UART_Send("INFO: emergency called by button press\r\n");
        	}
        }

        if (len > 0 && len < (int)sizeof(raw_values_buffer))
        {
        	snprintf(raw_values_buffer + len, sizeof(raw_values_buffer) - len,
        	                     "State: %s, Peak accel (candidate): %.2f m/s^2\r\n",
								 FallState_ToString(fall_state), peak_accel_candidate);
        }
//        UART_Send(raw_values_buffer);
//        UART_Send(norm_values_buffer);

        if ((HAL_GetTick() - last_sample) < SAMPLE_PERIOD_MS) {
        	HAL_Delay(SAMPLE_PERIOD_MS - (HAL_GetTick() - last_sample));
        }
        last_sample = HAL_GetTick();

        prev_accel_norm = accel_norm;

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

static void I2C1_Init(void)
{
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin       = GPIO_PIN_8 | GPIO_PIN_9;   // PB8=SCL, PB9=SDA
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;           // I2C must be open-drain
    GPIO_InitStruct.Pull      = GPIO_PULLUP;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    hi2c1.Instance				= I2C1;
    //hi2c1.Init.Timing          	= ((uint32_t)0x00702681);   // match BSP's I2C timing
    hi2c1.Init.Timing = 0x00100D14;
    hi2c1.Init.OwnAddress1     	= 0;
    hi2c1.Init.AddressingMode  	= I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode 	= I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     	= 0;
//    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode 	= I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode  	= I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK) { while (1) {} }
}

static void Buzzer_Init(void)
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* Start with buzzer off */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);
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
