#ifndef asdfdgdreuosii_includes_h
#define asdfdgdreuosii_includes_h

//#define HAL_USART_MODULE_ENABLED

#include  <stm32f7xx_hal.h>
#include <stm32f7xx_ll_dma.h>
#include  <cpu.h>
#include  <lib_mem.h>
#include  <os.h>
#include  <bsp_os.h>
#include  <bsp_clk.h>
#include  <bsp_led.h>
#include  <bsp_int.h>
#include  <app_cfg.h>


//stm32... includes
#define log_on_Pin GPIO_PIN_12
#define log_on_GPIO_Port GPIOB
#define our_3_Pin GPIO_PIN_13
#define our_3_GPIO_Port GPIOB
#define out_4_Pin GPIO_PIN_14
#define out_4_GPIO_Port GPIOB
#define out_1_Pin GPIO_PIN_3
#define out_1_GPIO_Port GPIOB
#define out_2_Pin GPIO_PIN_4
#define out_2_GPIO_Port GPIOB
#define rpm_input_Pin GPIO_PIN_5
#define rpm_input_GPIO_Port GPIOB
#define rpm_input_EXTI_IRQn EXTI9_5_IRQn
#define acc_button_Pin GPIO_PIN_6
#define acc_button_GPIO_Port GPIOB
#define acc_button_EXTI_IRQn EXTI9_5_IRQn
#define dec_button_Pin GPIO_PIN_7
#define dec_button_GPIO_Port GPIOB
#define dec_button_EXTI_IRQn EXTI9_5_IRQn


#endif
