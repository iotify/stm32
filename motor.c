/**
  ******************************************************************************
  * @author  IoTIFY
  * @brief   Texas Instruments DRV8834 stepper motor driver operation.
  ******************************************************************************
*/

#include "stm32f4xx.h"
#include "stm32f4xx_nucleo.h"

#define GPIO_PORT_DRV8834	GPIOA

#define GPIO_PIN_DRV8834_NENBL_AENBL	GPIO_PIN_15
#define GPIO_PIN_DRV8834_M0_APHASE		GPIO_PIN_10
#define GPIO_PIN_DRV8834_M1				GPIO_PIN_9
#define GPIO_PIN_DRV8834_CONFIG			GPIO_PIN_0
#define GPIO_PIN_DRV8834_NSLEEP			GPIO_PIN_11
#define GPIO_PIN_DRV8834_STEP_BENBL		GPIO_PIN_7
#define GPIO_PIN_DRV8834_DIR_BPHASE		GPIO_PIN_8
#define GPIO_PIN_DRV8834_NFAULT			GPIO_PIN_1

#define DRV8834_PIN_OUTPUT(mask)	\
	gpio_setup(GPIO_PORT_DRV8834, mask, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL)
#define DRV8834_PIN_INPUT(mask, pull)	\
	gpio_setup(GPIO_PORT_DRV8834, mask, GPIO_MODE_INPUT, pull)
#define DRV8834_PIN_SET(pin, value)	\
	HAL_GPIO_WritePin(GPIO_PORT_DRV8834, pin, value)
#define DRV8834_PIN_GET(pin)	\
	HAL_GPIO_ReadPin(GPIO_PORT_DRV8834, pin)

static UART_HandleTypeDef UartHandle;

static int uart_init(void)
{
	UartHandle.Instance = USART2;
	UartHandle.Init.BaudRate = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits = UART_STOPBITS_1;
	UartHandle.Init.Parity = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	return HAL_UART_Init(&UartHandle);
}

static int gpio_setup(GPIO_TypeDef *port, unsigned int pin_mask, int mode,
		int pull)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.Pin = pin_mask;
	GPIO_InitStruct.Mode = mode;
	GPIO_InitStruct.Pull = pull;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(port, &GPIO_InitStruct);
	return 0;
}

static int drv8834_init(int indexer_mode)
{
	__HAL_RCC_GPIOA_CLK_ENABLE();
	DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_NENBL_AENBL
			| GPIO_PIN_DRV8834_CONFIG | GPIO_PIN_DRV8834_NSLEEP
			| GPIO_PIN_DRV8834_STEP_BENBL | GPIO_PIN_DRV8834_DIR_BPHASE);
	DRV8834_PIN_INPUT(GPIO_PIN_DRV8834_NFAULT, GPIO_PULLUP);
	DRV8834_PIN_SET(GPIO_PIN_DRV8834_NSLEEP, GPIO_PIN_RESET);
	DRV8834_PIN_SET(GPIO_PIN_DRV8834_CONFIG,
			indexer_mode ? GPIO_PIN_SET : GPIO_PIN_RESET);
	DRV8834_PIN_SET(GPIO_PIN_DRV8834_NSLEEP, GPIO_PIN_SET);
	HAL_Delay(1);
	while (DRV8834_PIN_GET(GPIO_PIN_DRV8834_NFAULT) == GPIO_PIN_RESET)
		HAL_Delay(1);
	return 0;
}

static void drv8834_indexer_rotate(int steps)
{
	int microsteps_per_step = 4;
	int microsteps;
	int i;

	DRV8834_PIN_SET(GPIO_PIN_DRV8834_NENBL_AENBL, GPIO_PIN_RESET);
	if (steps < 0) {
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_DIR_BPHASE, GPIO_PIN_RESET);
		steps = -steps;
	} else
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_DIR_BPHASE, GPIO_PIN_SET);
	microsteps = steps * microsteps_per_step;
	switch (microsteps_per_step) {
	case 1:
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M1);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M1, GPIO_PIN_RESET);
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M0_APHASE);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_RESET);
		break;
	case 2:
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M1);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M1, GPIO_PIN_RESET);
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M0_APHASE);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_SET);
		break;
	case 4:
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M1);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M1, GPIO_PIN_RESET);
		DRV8834_PIN_INPUT(GPIO_PIN_DRV8834_M0_APHASE, GPIO_NOPULL);
		break;
	case 8:
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M1);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M1, GPIO_PIN_SET);
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M0_APHASE);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_RESET);
		break;
	case 16:
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M1);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M1, GPIO_PIN_SET);
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M0_APHASE);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_SET);
		break;
	case 32:
		DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M1);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_M1, GPIO_PIN_SET);
		DRV8834_PIN_INPUT(GPIO_PIN_DRV8834_M0_APHASE, GPIO_NOPULL);
		break;
	}
	for (i = 0; i < microsteps; i++) {
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_SET);
		HAL_Delay(50);
		DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_RESET);
		HAL_Delay(50);
	}
}

static void drv8834_phase_enable_rotate(int steps)
{
	int electr_angle;
	int microsteps = (steps > 0) ? (steps * 2) : (-steps * 2);
	int i;

	DRV8834_PIN_OUTPUT(GPIO_PIN_DRV8834_M0_APHASE);
	electr_angle = 45;
	DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_SET);
	DRV8834_PIN_SET(GPIO_PIN_DRV8834_NENBL_AENBL, GPIO_PIN_SET);
	DRV8834_PIN_SET(GPIO_PIN_DRV8834_DIR_BPHASE, GPIO_PIN_SET);
	DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_SET);

	for (i = 0; i < microsteps; i++) {
		if (steps > 0) {
			electr_angle += 45;
			if (electr_angle > 180)
				electr_angle -= 360;
		} else {
			electr_angle -= 45;
			if (electr_angle < -180)
				electr_angle += 360;
		}
		switch(electr_angle) {
		case 0:
		case 180:
		case -180:
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_RESET);
			break;
		case 90:
		case -90:
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_NENBL_AENBL, GPIO_PIN_RESET);
			break;
		case 45:
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_NENBL_AENBL, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_DIR_BPHASE, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_SET);
			break;
		case 135:
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_RESET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_NENBL_AENBL, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_DIR_BPHASE, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_SET);
			break;
		case -135:
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_RESET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_NENBL_AENBL, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_DIR_BPHASE, GPIO_PIN_RESET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_SET);
			break;
		case -45:
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_M0_APHASE, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_NENBL_AENBL, GPIO_PIN_SET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_DIR_BPHASE, GPIO_PIN_RESET);
			DRV8834_PIN_SET(GPIO_PIN_DRV8834_STEP_BENBL, GPIO_PIN_SET);
			break;
		}
		HAL_Delay(100);
	}
}

int main(void)
{
	int steps = 10;
	int indexer_mode = 1;

	HAL_Init();
	uart_init();
	drv8834_init(indexer_mode);
	while (1)
	{
	printf("DRV8834 motor rotation by %d steps\n", steps);
	if (indexer_mode)
		drv8834_indexer_rotate(steps);
	else
		drv8834_phase_enable_rotate(steps);
	
	HAL_Delay(5000);	
	}
	return 0;
}

int __io_putchar(int ch)
{
	HAL_UART_Transmit(&UartHandle, (uint8_t *)&ch, 1, 0xFFFF);
	return ch;
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}
