#ifndef _STM32F1XX_EAC_UART_H
#define	_STM32F1XX_EAC_UART_H

#define EAC_FLAG_IS_EMPTY 0b00000001
#define EAC_FLAG_IS_FULL 0b00000010
#define EAC_FLAG_IS_OVERRUN 0b00000100
#define EAC_FLAG_INIT	0b00000001

#define EAC_SET_FLAG(_statusRegister_,_flag_)  do { (_statusRegister_) |= (_flag_); } while(0)
#define EAC_CLEAR_FLAG(_statusRegister_,_flag_)  do { (_statusRegister_) &= ~(_flag_); } while(0)
#define EAC_TEST_FLAG(_statusRegister_,_flag_) ((_statusRegister_) & (_flag_))

#define	EAC_CIRCULARBUFFER_INIT(__buff__) {.buff=(__buff__), .read_index=0, .write_index=0, .status=EAC_FLAG_INIT}

#include "stm32f1xx_hal.h"

typedef struct EAC_CircularBuffer
{
	uint16_t buffDimension;
	uint8_t* buff;
	volatile uint16_t read_index;
	volatile uint16_t write_index;
	volatile uint8_t status;
} EAC_CircularBuffer_t;

HAL_StatusTypeDef EAC_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);

HAL_StatusTypeDef EAC_UART_Start_Rx(UART_HandleTypeDef *huart, uint8_t dimensionBits);

HAL_StatusTypeDef EAC_UART_Stop_Rx(UART_HandleTypeDef *huart);

int EAC_UART_DequeueRxByte(UART_HandleTypeDef *huart, uint8_t* rxByte);

void EAC_UART_IRQHandler(UART_HandleTypeDef *huart);

#endif
