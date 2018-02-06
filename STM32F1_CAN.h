// -------------------------------------------------------------
// a simple Arduino STM32F103C8 CAN driver
// by jiauka
// 
//
#ifndef __STM32F1_CAN_H__
#define __STM32F1_CAN_H__

#include <Arduino.h>

//#define INTTX
#define NEW_LIB // use modified lib, see https://github.com/jiauka/STM32F1_CAN

#if !defined(SIZE_RX_BUFFER)
#define SIZE_RX_BUFFER  16 // receive incoming ring buffer default size
#endif

#if !defined(SIZE_TX_BUFFER)
#define SIZE_TX_BUFFER  16 // transmit ring buffer default size
#endif
#define NUM_BUFFERED_MBOXES 2

class STM32F1_CAN {
public:
	typedef struct CanMsgTypeDef {
		uint32_t id; /*!< Specifies the extended identifier.
		 This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

		uint32_t len; /*!< Specifies the length of the frame
		 This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

		uint8_t Data[8]; /*!< Contains the data
		 This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

	} CanMsgTypeDef;

	typedef struct RingbufferTypeDef {
		volatile uint16_t head;
		volatile uint16_t tail;
		uint16_t size;
		volatile STM32F1_CAN::CanMsgTypeDef *buffer;
	} RingbufferTypeDef;

protected:
	uint16_t sizeRxBuffer;
#ifndef NEW_LIB 
	uint16_t sizeTxBuffer;
#else
	uint16_t sizeTxBuffer[NUM_BUFFERED_MBOXES];
#endif
public:

	STM32F1_CAN();
	static STM32F1_CAN& getInstance();
	void begin(bool UseAltPins);
	bool write(const CanMsgTypeDef &msg, bool wait_sent);
	bool write(const CanMsgTypeDef &msg, bool wait_sent, uint8_t n);
	bool read(CanMsgTypeDef &msg);

	// Before begin, you can define rx buffer size. Default is SIZE_RX_BUFFER. This does not have effect after begin.
	void setRxBufferSize(uint16_t size) {
		if (!isInitialized())
			sizeRxBuffer = size;
	}
#ifndef NEW_LIB
	// Before begin, you can define global tx buffer size. Default is SIZE_TX_BUFFER. This does not have effect after begin.
	void setTxBufferSize(uint16_t size) {if (!isInitialized() ) sizeTxBuffer=size;}
#else
	// You can define mailbox specific tx buffer size. This can be defined only once per mailbox.
	// As default prioritized messages will not be buffered. If you define buffer size for mail box, the messages will be
	// buffered to own buffer, if necessary.
	void setMailBoxTxBufferSize(uint8_t mbox, uint16_t size);
#endif
	bool addToRingBuffer(RingbufferTypeDef &ring, const CanMsgTypeDef &msg);
	bool removeFromRingBuffer(RingbufferTypeDef &ring, CanMsgTypeDef &msg);
private:

	bool isInitialized() {
		return rx_buffer != 0;
	}
	void initRingBuffer(RingbufferTypeDef &ring, volatile CanMsgTypeDef *buffer,
			uint32_t size);
	void initializeBuffers(void);
	bool isRingBufferEmpty(RingbufferTypeDef &ring);
	uint32_t ringBufferCount(RingbufferTypeDef &ring);
public:
	RingbufferTypeDef rxRing;
#ifndef NEW_LIB 
	RingbufferTypeDef txRing;
#else
	RingbufferTypeDef txRing1;
	RingbufferTypeDef txRing2;
#endif
private:
	volatile CanMsgTypeDef *rx_buffer;
#ifndef NEW_LIB   
	volatile CanMsgTypeDef *tx_buffer;
#else
	volatile CanMsgTypeDef *tx_buffer0;
	volatile CanMsgTypeDef *tx_buffer1;
#endif
};

//extern STM32F1_CAN Can0;

#endif
