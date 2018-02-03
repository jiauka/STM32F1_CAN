#include "STM32F1_CAN.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"

static CAN_HandleTypeDef CanHandle;

STM32F1_CAN::STM32F1_CAN() {
	numTxMailboxes=NUM_MAILBOXES;
	sizeRxBuffer=SIZE_RX_BUFFER;
	sizeTxBuffer=SIZE_TX_BUFFER;
    rx_buffer=0;
    tx_buffer=0;
   // Initialize all message box spesific ring buffers to 0.
    for(int i=0; i<getNumMailBoxes(); i++) {
      txRings[i]=0;
    }
}

STM32F1_CAN& STM32F1_CAN::getInstance()
{
    static STM32F1_CAN theOneAndOnly;

    return theOneAndOnly;
}


void STM32F1_CAN::begin(bool UseAltPins) {
	static CanTxMsgTypeDef TxMessage;
	static CanRxMsgTypeDef RxMessage;
	CAN_FilterConfTypeDef sFilterConfig;
	GPIO_InitTypeDef GPIO_InitStruct;
	initializeBuffers();
	__HAL_RCC_CAN1_CLK_ENABLE();

	/* Enable GPIO clock */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	if(UseAltPins) {
		/* Enable AFIO clock and remap CAN PINs to PB_8 and PB_9*/
		__HAL_RCC_AFIO_CLK_ENABLE();
		__HAL_AFIO_REMAP_CAN1_2();

		/* CAN1 RX GPIO pin configuration */
		GPIO_InitStruct.Pin = GPIO_PIN_8;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	} 
	else {
		__HAL_RCC_AFIO_CLK_ENABLE();
		/* CAN1 RX GPIO pin configuration */
		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	}
	/* Configure CAN1 **************************************************/
	/* Struct init*/
	CanHandle.Instance = CAN1;
	CanHandle.pTxMsg = &TxMessage;
	CanHandle.pRxMsg = &RxMessage;

	CanHandle.Init.TTCM = DISABLE;
	CanHandle.Init.ABOM = DISABLE;
	CanHandle.Init.AWUM = DISABLE;
	CanHandle.Init.NART = DISABLE;
	CanHandle.Init.RFLM = DISABLE;
	CanHandle.Init.TXFP = ENABLE; // JCB ENABLE  ;
	CanHandle.Init.Mode = CAN_MODE_NORMAL;
	CanHandle.Init.SJW = CAN_SJW_1TQ;
	CanHandle.Init.BS1 = CAN_BS1_3TQ;
	CanHandle.Init.BS2 = CAN_BS2_5TQ;
	CanHandle.Init.Prescaler = 16;

	/*Initializes the CAN1 */
	HAL_CAN_Init(&CanHandle);

	/* CAN filter init */
	sFilterConfig.FilterNumber = 0;
	sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	sFilterConfig.FilterIdHigh = 0x0000;
	sFilterConfig.FilterIdLow = 0x0000;
	sFilterConfig.FilterMaskIdHigh = 0x0000;
	sFilterConfig.FilterMaskIdLow = 0x0000;
	sFilterConfig.FilterFIFOAssignment = CAN_FIFO0;
	sFilterConfig.FilterActivation = ENABLE;
	sFilterConfig.BankNumber = 0;
	if(HAL_CAN_ConfigFilter(&CanHandle, &sFilterConfig) != HAL_OK) {
		__HAL_CAN_ENABLE_IT(&CanHandle, CAN_IT_FMP0);
		/* Filter configuration Error */
//      Error_Handler();
	}
	HAL_CAN_Receive_IT(&CanHandle, CAN_FIFO0);
//    		__HAL_CAN_ENABLE_IT(&CanHandle, CAN_IT_FMP0);
	HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);

#ifdef INTTX
	HAL_NVIC_SetPriority(CAN1_TX_IRQn, 5, 0);
	HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
	__HAL_CAN_ENABLE_IT(&CanHandle, CAN_IT_TME);
#endif

}

bool STM32F1_CAN::write(const CanMsgTypeDef &msg,bool wait_sent) {
	bool ret=true;
	
	CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
	CanHandle.pTxMsg->DLC = msg.len;
	CanHandle.pTxMsg->IDE = CAN_ID_EXT;
	CanHandle.pTxMsg->ExtId = msg.id;
	for(uint32_t i = 0; i < msg.len; i++)
		CanHandle.pTxMsg->Data[i] = msg.Data[i];
	if ( wait_sent ) {
		if(HAL_CAN_Transmit(&CanHandle, 100) != HAL_OK) {
			ret= false;
		}
	}
	else
	{
		uint8_t prio = (uint8_t) ((msg.id >> 26) & 0x7);
		if(HAL_CAN_Transmit_IT(&CanHandle) !=HAL_OK) {
			if(STM32F1_CAN::getInstance().addToRingBuffer(STM32F1_CAN::getInstance().txRing, msg)==false) {
				ret= false;; // no more room
			}
		}
	}
#if 0
	CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
	CanHandle.pTxMsg->DLC = msg.len;
	CanHandle.pTxMsg->IDE = CAN_ID_EXT;
	CanHandle.pTxMsg->ExtId = msg.id;
	for(uint32_t i = 0; i < msg.len; i++)
	CanHandle.pTxMsg->Data[i] = msg.Data[i];

	if(HAL_CAN_Transmit(&CanHandle, 10) != HAL_OK) {
		ret= false;
	}
	else
	ret= true;
#endif	
	__HAL_CAN_ENABLE_IT(&CanHandle, CAN_IT_FMP0);
	return ret;
}

bool STM32F1_CAN::read(CanMsgTypeDef &msg) {
    return removeFromRingBuffer(rxRing, msg);
}

void STM32F1_CAN::initializeBuffers() {
    if( isInitialized() )
    	return;
  
    // set up the transmit and receive ring buffers
    if(tx_buffer==0)
    	tx_buffer=new CanMsgTypeDef[sizeTxBuffer];
    initRingBuffer(txRing, tx_buffer, sizeTxBuffer);
    if(rx_buffer==0)
    	rx_buffer=new CanMsgTypeDef[sizeRxBuffer];

    initRingBuffer(rxRing, rx_buffer, sizeRxBuffer);
}
void STM32F1_CAN::initRingBuffer(RingbufferTypeDef &ring, volatile CanMsgTypeDef *buffer, uint32_t size)
{
    ring.buffer = buffer;
    ring.size = size;
    ring.head = 0;
    ring.tail = 0;
}
bool STM32F1_CAN::addToRingBuffer (RingbufferTypeDef &ring, const CanMsgTypeDef &msg)
{
    uint16_t nextEntry;

    nextEntry =(ring.head + 1) % ring.size;

    /* check if the ring buffer is full */

    if(nextEntry == ring.tail) {
        return(false);
    }

    /* add the element to the ring */

    memcpy((void *)&ring.buffer[ring.head],(void *)&msg, sizeof(CanMsgTypeDef));

    /* bump the head to point to the next free entry */

    ring.head = nextEntry;

    return(true);
}

/*
 * \brief Remove a CAN message from the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 * \param msg - message structure to fill in.
 *
 * \retval true if a message was removed, false if the ring is empty.
 *
 */

bool STM32F1_CAN::removeFromRingBuffer(RingbufferTypeDef &ring, CanMsgTypeDef &msg)
{

    /* check if the ring buffer has data available */

    if(isRingBufferEmpty(ring) == true) {
        return(false);
    }

    /* copy the message */

    memcpy((void *)&msg,(void *)&ring.buffer[ring.tail], sizeof(CanMsgTypeDef));

    /* bump the tail pointer */

    ring.tail =(ring.tail + 1) % ring.size;

    return(true);
}

/*
 * \brief Check if the specified ring buffer is empty.
 *
 * \param ring - ring buffer to use.
 *
 * \retval true if the ring contains data, false if the ring is empty.
 *
 */

bool STM32F1_CAN::isRingBufferEmpty(RingbufferTypeDef &ring)
{
    if(ring.head == ring.tail) {
        return(true);
    }

    return(false);
}

/*
 * \brief Count the number of entries in the specified ring buffer.
 *
 * \param ring - ring buffer to use.
 *
 * \retval a count of the number of elements in the ring buffer.
 *
 */

uint32_t STM32F1_CAN::ringBufferCount(RingbufferTypeDef &ring)
{
    int32_t entries;

    entries = ring.head - ring.tail;

    if(entries < 0) {
        entries += ring.size;
    }

    return((uint32_t)entries);
}


extern "C" void CAN1_RX0_IRQHandler(void) {
	HAL_CAN_IRQHandler(&CanHandle);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _canHandle) {
	CanMsgTypeDef msg;
	msg.id=_canHandle->pRxMsg->ExtId;
	msg.len=_canHandle->pRxMsg->DLC;
	memcpy(msg.Data,_canHandle->pRxMsg->Data,msg.len);
    STM32F1_CAN::getInstance().addToRingBuffer(STM32F1_CAN::getInstance().rxRing, msg);
    if (HAL_CAN_Receive_IT(_canHandle, CAN_FIFO0) != HAL_OK)
    {
      __HAL_CAN_ENABLE_IT(_canHandle, CAN_IT_FMP0);  // set interrupt flag for RX FIFO0 if CAN locked
    }
}


extern "C" void CAN1_TX_IRQHandler(void) {
	HAL_CAN_IRQHandler(&CanHandle);
}

void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* _canHandle)
{
	CanMsgTypeDef frame;

    if(STM32F1_CAN::getInstance().removeFromRingBuffer(STM32F1_CAN::getInstance().txRing, frame)) {
		CanHandle.pTxMsg->RTR = CAN_RTR_DATA;
		CanHandle.pTxMsg->DLC = frame.len;
		CanHandle.pTxMsg->IDE = CAN_ID_EXT;
		CanHandle.pTxMsg->ExtId = frame.id;
		for(uint32_t i = 0; i < frame.len; i++)
			CanHandle.pTxMsg->Data[i] = frame.Data[i];
		if(HAL_CAN_Transmit_IT(&CanHandle) !=HAL_OK) {
			//Serial3.println("error");
		}
	}
}
