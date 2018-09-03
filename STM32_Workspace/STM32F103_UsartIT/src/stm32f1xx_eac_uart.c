#include "stm32f1xx_eac_uart.h"
#include "stm32f1xx_hal.h"

static HAL_StatusTypeDef _UART_Transmit_IT(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef _UART_EndTransmit_IT(UART_HandleTypeDef *huart);
static HAL_StatusTypeDef _UART_Receive_IT(UART_HandleTypeDef *huart);

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  pData: Pointer to data buffer
  * @param  Size: Amount of data to be sent
  * @retval HAL status
  */
HAL_StatusTypeDef EAC_UART_Transmit_IT(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  uint32_t tmp_state = 0;

  tmp_state = huart->State;
  if((tmp_state == HAL_UART_STATE_READY) || (tmp_state == HAL_UART_STATE_BUSY_RX))
  {
    if((pData == NULL ) || (Size == 0))
    {
      return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pTxBuffPtr = pData;
    huart->TxXferSize = Size;
    huart->TxXferCount = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* Check if a receive process is ongoing or not */
    if(huart->State == HAL_UART_STATE_BUSY_RX)
    {
      huart->State = HAL_UART_STATE_BUSY_TX_RX;
    }
    else
    {
      huart->State = HAL_UART_STATE_BUSY_TX;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART Transmit data register empty Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_TXE);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Configs the reception of data in interrupt mode.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @param  pData: Pointer to circular buffer
  * @retval HAL status
  */
HAL_StatusTypeDef EAC_UART_Receive_IT(UART_HandleTypeDef *huart,
		EAC_CircularBuffer_t *pCircularBuffer)
{
  uint32_t tmp_state = 0;

  tmp_state = huart->State;
  if((tmp_state == HAL_UART_STATE_READY) || (tmp_state == HAL_UART_STATE_BUSY_TX))
  {
    if(pCircularBuffer == NULL)
    {
      return HAL_ERROR;
    }
    if(pCircularBuffer->buff == NULL)
    {
    	return HAL_ERROR;
    }

    /* Process Locked */
    __HAL_LOCK(huart);

    huart->pRxBuffPtr = pCircularBuffer;
    //huart->RxXferSize = Size;
    //huart->RxXferCount = Size;

    huart->ErrorCode = HAL_UART_ERROR_NONE;
    /* Check if a transmit process is ongoing or not */
    if(huart->State == HAL_UART_STATE_BUSY_TX)
    {
      huart->State = HAL_UART_STATE_BUSY_TX_RX;
    }
    else
    {
      huart->State = HAL_UART_STATE_BUSY_RX;
    }

    /* Process Unlocked */
    __HAL_UNLOCK(huart);

    /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);

    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  This function handles UART interrupt request.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval None
  */
void EAC_UART_IRQHandler(UART_HandleTypeDef *huart)
{
  uint32_t tmp_flag = 0, tmp_it_source = 0;

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_PE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_PE);
  /* UART parity error interrupt occurred ------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    huart->ErrorCode |= HAL_UART_ERROR_PE;
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_FE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_ERR);
  /* UART frame error interrupt occurred -------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    huart->ErrorCode |= HAL_UART_ERROR_FE;
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_NE);
  /* UART noise error interrupt occurred -------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    huart->ErrorCode |= HAL_UART_ERROR_NE;
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_ORE);
  /* UART Over-Run interrupt occurred ----------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
    huart->ErrorCode |= HAL_UART_ERROR_ORE;
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE);
  /* UART in mode Receiver ---------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
	  /* Our routine for buffer */
	  _UART_Receive_IT(huart);
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TXE);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TXE);
  /* UART in mode Transmitter ------------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
	  _UART_Transmit_IT(huart);
  }

  tmp_flag = __HAL_UART_GET_FLAG(huart, UART_FLAG_TC);
  tmp_it_source = __HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC);
  /* UART in mode Transmitter end --------------------------------------------*/
  if((tmp_flag != RESET) && (tmp_it_source != RESET))
  {
	  _UART_EndTransmit_IT(huart);
  }

  if(huart->ErrorCode != HAL_UART_ERROR_NONE)
  {
    /* Clear all the error flag at once */
    __HAL_UART_CLEAR_PEFLAG(huart);

    /* Set the UART state ready to be able to start again the process */
    huart->State = HAL_UART_STATE_READY;

    HAL_UART_ErrorCallback(huart);
  }
}

/**
  * @brief  Receives an amount of data in non blocking mode
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef _UART_Receive_IT(UART_HandleTypeDef *huart)
{
  uint16_t* tmp;
  uint32_t tmp_state = 0;

  tmp_state = huart->State;
  if((tmp_state == HAL_UART_STATE_BUSY_RX) || (tmp_state == HAL_UART_STATE_BUSY_TX_RX))
  {
    if(huart->Init.WordLength == UART_WORDLENGTH_9B)
    {
//      tmp = (uint16_t*) huart->pRxBuffPtr;
//      if(huart->Init.Parity == UART_PARITY_NONE)
//      {
//        *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x01FF);
//        huart->pRxBuffPtr += 2;
//      }
//      else
//      {
//        *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FF);
//        huart->pRxBuffPtr += 1;
//      }
    	return HAL_ERROR;
    }
    else
    {
      if(huart->Init.Parity == UART_PARITY_NONE)
      {
    	  EAC_CircularBuffer_t* circBuff = (EAC_CircularBuffer_t*)(*huart->pRxBuffPtr);
    	  uint16_t *read_idx = &(circBuff->read_index);
    	  uint16_t *write_idx = &(circBuff->write_index);
    	  uint8_t *buff_ptr = (circBuff->buff);
    	  uint8_t *status = &(circBuff->status);

    	  //check if already full
    	  if(!(*status & EAC_FLAG_IS_FULL))
    	  {
        	  buff_ptr[*write_idx] =
        			  (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
        	  (*write_idx)++;
        	  (*write_idx)&=(sizeof(buff_ptr) - 1);
        	  //clear empty flag
        	  *status &= !EAC_FLAG_IS_EMPTY;

        	  //if full
        	  if((*write_idx) == (*read_idx))
        	  {
        		  *status |= EAC_FLAG_IS_FULL;
        	  }
    	  }
    	  else //if already full, overrun
    	  {
    		  *status |= EAC_FLAG_IS_OVERRUN;
    	  }

      }
      else
      {
//        *huart->pRxBuffPtr++ = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
    	  return HAL_ERROR;
      }
    }

    // __HAL_UART_DISABLE_IT(huart, UART_IT_RXNE);
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}

/**
  * @brief  Sends an amount of data in non blocking mode.
  * @param  huart: Pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef _UART_Transmit_IT(UART_HandleTypeDef *huart)
{
  uint16_t* tmp;
  uint32_t tmp_state = 0;

  tmp_state = huart->State;
  if((tmp_state == HAL_UART_STATE_BUSY_TX) || (tmp_state == HAL_UART_STATE_BUSY_TX_RX))
  {
    if(huart->Init.WordLength == UART_WORDLENGTH_9B)
    {
      tmp = (uint16_t*) huart->pTxBuffPtr;
      huart->Instance->DR = (uint16_t)(*tmp & (uint16_t)0x01FF);
      if(huart->Init.Parity == UART_PARITY_NONE)
      {
        huart->pTxBuffPtr += 2;
      }
      else
      {
        huart->pTxBuffPtr += 1;
      }
    }
    else
    {
      huart->Instance->DR = (uint8_t)(*huart->pTxBuffPtr++ & (uint8_t)0x00FF);
    }

    if(--huart->TxXferCount == 0)
    {
      /* Disable the UART Transmit Complete Interrupt */
      __HAL_UART_DISABLE_IT(huart, UART_IT_TXE);

      /* Enable the UART Transmit Complete Interrupt */
      __HAL_UART_ENABLE_IT(huart, UART_IT_TC);
    }
    return HAL_OK;
  }
  else
  {
    return HAL_BUSY;
  }
}


/**
  * @brief  Wraps up transmission in non blocking mode.
  * @param  huart: pointer to a UART_HandleTypeDef structure that contains
  *                the configuration information for the specified UART module.
  * @retval HAL status
  */
static HAL_StatusTypeDef _UART_EndTransmit_IT(UART_HandleTypeDef *huart)
{
  /* Disable the UART Transmit Complete Interrupt */
  __HAL_UART_DISABLE_IT(huart, UART_IT_TC);

  /* Check if a receive process is ongoing or not */
  if(huart->State == HAL_UART_STATE_BUSY_TX_RX)
  {
    huart->State = HAL_UART_STATE_BUSY_RX;
  }
  else
  {
    huart->State = HAL_UART_STATE_READY;
  }

  HAL_UART_TxCpltCallback(huart);

  return HAL_OK;
}
