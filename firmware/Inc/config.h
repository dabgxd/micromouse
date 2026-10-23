/* USER CODE BEGIN Header */
/**
 * @file   config.h
 * @brief  Central configuration for pin definitions, constants and hardware parameters.
 *         All user‑defined macros should be placed here to keep code portable and
 *         survive CubeMX regeneration.
 */
/* USER CODE END Header */

#ifndef CONFIG_H_
#define CONFIG_H_

/* ----------------------- GPIO Pin Definitions ----------------------- */
/* Buttons */
#define BTN_KEY_GPIO_Port        GPIOA
#define BTN_KEY_Pin              GPIO_PIN_0   // Start / reset button (input, pull‑up)

/* LED */
#define LED_ONBOARD_GPIO_Port    GPIOC
#define LED_ONBOARD_Pin          GPIO_PIN_13

/* Motor Driver TB6612FNG */
#define PWMA_GPIO_Port           GPIOA
#define PWMA_Pin                 GPIO_PIN_8   // TIM1_CH1 PWM
#define PWMB_GPIO_Port           GPIOA
#define PWMB_Pin                 GPIO_PIN_9   // TIM1_CH2 PWM
#define AIN1_GPIO_Port           GPIOB
#define AIN1_Pin                 GPIO_PIN_12
#define AIN2_GPIO_Port           GPIOB
#define AIN2_Pin                 GPIO_PIN_13
#define BIN1_GPIO_Port           GPIOB
#define BIN1_Pin                 GPIO_PIN_14
#define BIN2_GPIO_Port           GPIOB
#define BIN2_Pin                 GPIO_PIN_15
#define STBY_GPIO_Port           GPIOA
#define STBY_Pin                 GPIO_PIN_10  // Standby (high = enable)

/* VL53L0X XSHUT pins (5 sensors) */
#define XSHUT_LEFT_90_GPIO_Port  GPIOA
#define XSHUT_LEFT_90_Pin        GPIO_PIN_1
#define XSHUT_DIAG_LEFT_45_GPIO_Port GPIOA
#define XSHUT_DIAG_LEFT_45_Pin   GPIO_PIN_4
#define XSHUT_FRONT_0_GPIO_Port  GPIOA
#define XSHUT_FRONT_0_Pin        GPIO_PIN_5
#define XSHUT_DIAG_RIGHT_45_GPIO_Port GPIOB
#define XSHUT_DIAG_RIGHT_45_Pin  GPIO_PIN_0
#define XSHUT_RIGHT_90_GPIO_Port GPIOB
#define XSHUT_RIGHT_90_Pin       GPIO_PIN_1

/* MPU6050 */
#define MPU_INT_GPIO_Port        GPIOB
#define MPU_INT_Pin              GPIO_PIN_5   // EXTI5

/* HC‑05 UART */
#define HC05_TX_GPIO_Port        GPIOA
#define HC05_TX_Pin              GPIO_PIN_2   // USART2_TX -> HC‑05_RXD
#define HC05_RX_GPIO_Port        GPIOA
#define HC05_RX_Pin              GPIO_PIN_3   // USART2_RX <- HC‑05_TXD

/* ----------------------- Physical Constants ----------------------- */
#define WHEEL_DIAMETER_MM        24.0f      // measure actual wheel
#define ENCODER_CPR              200        // pulses per revolution (motor shaft)
#define GEAR_RATIO               1.0f       // if gear reduction, set >1
#define MM_PER_TICK ((3.14159f * WHEEL_DIAMETER_MM) / (ENCODER_CPR * GEAR_RATIO * 4.0f))

/* ----------------------- Communication ----------------------- */
#define UART_BAUDRATE            115200

#endif // CONFIG_H_

