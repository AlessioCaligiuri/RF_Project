/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"
#include "stm32f1xx_eac_uart.h"
#include "nrf24l01.h"
#include <stdbool.h>
#include "sim_parser.h"
#include <string.h>

#define	USART1_BUFFER_LENGTH	512
const char SIM800_AT[]=			"AT\r\n";
#define SIM800_CMGF1			"AT+CMGF=1\r\n"
#define SIM800_CNMI				"AT+CNMI=1,2,0,0,0\r\n"
#define SIM800_CMD_RX_MSG		"CMT"

#define NUMBER_FIELD_LENGTH					16
#define NUMBER_AUTHORIZED_PHONE_NUMBERS		2

#define NRF24_PACKET_LENGTH		32

void SystemClock_Config(void);
void Error_Handler(void);

static SIM_Parser_t simParser;
static uint8_t rxByte_fromPC = 0;
static uint8_t rxByte_fromSIM800 = 0;
static volatile bool is_txToPC_Completed = 0;
static volatile bool is_txToSIM800_Completed = 0;

/* Data for NRF24L01 module */
static const uint8_t rx_address[5] = {1, 2, 3, 4, 5};
static const uint8_t tx_address[5] = {1, 2, 3, 4, 6};
nrf24l01 nrf;

/* Authorized phone numbers array */
const char listNumber[NUMBER_AUTHORIZED_PHONE_NUMBERS][NUMBER_FIELD_LENGTH] =
				{"\"+393803415931\"",
				 "\"+393487222907\""};

/* Function for 2.4GHz module init */
void nrf24l01_setup()
{
	uint32_t rx_data;
	nrf24l01_config config;
	config.data_rate        = NRF_DATA_RATE_1MBPS;
	config.tx_power         = NRF_TX_PWR_0dBm;
	config.crc_width        = NRF_CRC_WIDTH_1B;
	config.addr_width       = NRF_ADDR_WIDTH_5;
	config.payload_length   = NRF24_PACKET_LENGTH;    // maximum is 32 bytes
	config.retransmit_count = 15;   // maximum is 15 times
	config.retransmit_delay = 0x0F; // 4000us, LSB:250us
	config.rf_channel       = 0;
	config.rx_address       = rx_address;
	config.tx_address       = tx_address;
	config.rx_buffer        = (uint8_t*)&rx_data;

	config.spi         = &hspi1;
	config.spi_timeout = 10; // milliseconds
	config.ce_port     = NRF_CE_GPIO_Port;
	config.ce_pin      = NRF_CE_Pin;
	config.irq_port    = NRF_IRQ_GPIO_Port;
	config.irq_pin     = NRF_IRQ_Pin;
	config.csn_port	 = NRF_CSN_GPIO_Port;
	config.csn_pin	 = NRF_CSN_Pin;

	nrf_init(&nrf, &config);
 }

/* Callback for NRF24 IRQ */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	nrf_irq_handler(&nrf);
}

/* Callback for UART transmission completed */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart)
{
	if(huart == &huart1)
		//is_txToPC_Completed = 1;
		is_txToSIM800_Completed = 1;
//	if(huart == &huart2)
//		is_txToSIM800_Completed = 1;

	return;
}

/* Configures SIM800 to receive and show sms */
void SIM800_InitSMSReception(UART_HandleTypeDef* huart)
{
	/* Send AT command for the autobaudrate */
	EAC_UART_Transmit_IT(huart,SIM800_AT,strlen(SIM800_AT));
	while(!is_txToSIM800_Completed);
	is_txToSIM800_Completed = false;

	HAL_Delay(500);

	/* Set text mode */
	EAC_UART_Transmit_IT(huart,SIM800_CMGF1,strlen(SIM800_CMGF1));
	while(!is_txToSIM800_Completed);
	is_txToSIM800_Completed = false;

	HAL_Delay(500);

	// TODO: Check SIM800 response

	/* Set sms preview and disable the sms storing */
	EAC_UART_Transmit_IT(huart,SIM800_CNMI,strlen(SIM800_CNMI));
	while(!is_txToSIM800_Completed);
	is_txToSIM800_Completed = false;

	HAL_Delay(500);
}

/* Debug function to send to PC all the received fields */
void sendFieldsToPC()
{
	  EAC_UART_Transmit_IT(&huart1,simParser.cmd,strlen(simParser.cmd));
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,"\r\n",2);
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,simParser.field1,strlen(simParser.field1));
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,"\r\n",2);
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,simParser.field2,strlen(simParser.field2));
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,"\r\n",2);
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,simParser.field3,strlen(simParser.field3));
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,"\r\n",2);
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,simParser.text,strlen(simParser.text));
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
	  EAC_UART_Transmit_IT(&huart1,"\r\n",2);
	  while(!is_txToPC_Completed);
	  is_txToPC_Completed = false;
}

int main(void)
{
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */
  nrf24l01_setup();
  SIM_Parser_init(&simParser);
  EAC_UART_Start_Rx(&huart1,9);
  SIM800_InitSMSReception(&huart1);
  /* USER CODE END 2 */

  /* Infinite loop */
  while (1)
  {

	  nrf_send_packet_noack(&nrf, "a");
	  continue;

	  while(EAC_UART_DequeueRxByte(&huart1,&rxByte_fromSIM800))
	  {
		  if(simParser.isCompleted)
			  break;

		  SIM_Parser_update(&simParser,rxByte_fromSIM800);
	  }

	  /* Semantic parser */
	  if(simParser.isCompleted)
	  {
		  simParser.isCompleted = false;
		  //sendFieldsToPC();

		  /* Check if the received AT response is a new message */
		  if(!strcmp(simParser.cmd, SIM800_CMD_RX_MSG))
		  {
			  bool isAuthorizedNumber = false;
			  /* Check if message is from an authorized number */
			  for (int i=0; i<NUMBER_AUTHORIZED_PHONE_NUMBERS; i++)
			  {
				  if(!strcmp(simParser.field1, listNumber[i]))
				  {
					  isAuthorizedNumber = true;
					  break;
				  }
			  }
			  if(isAuthorizedNumber)
			  {
				  char textToSend[NRF24_PACKET_LENGTH] = {0};
				  strncpy (textToSend, simParser.text, NRF24_PACKET_LENGTH);
				  nrf_send_packet(&nrf, textToSend);
			  }
		  }
	  }
  }
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
