// -------------------------------------------------------------
// a simple Arduino STM32F103C8 CAN driver
// by jiauka
// 
//
#ifndef __STM32F1_CAN_H__
#define __STM32F1_CAN_H__

#include <Arduino.h>
#include "stm32f1xx_hal.h"



typedef struct CanMsgTypeDef
{
  uint32_t id;    /*!< Specifies the extended identifier.
                         This parameter must be a number between Min_Data = 0 and Max_Data = 0x1FFFFFFF */

  uint32_t len;      /*!< Specifies the length of the frame
                          This parameter must be a number between Min_Data = 0 and Max_Data = 8 */

  uint8_t Data[8];   /*!< Contains the data
                          This parameter must be a number between Min_Data = 0 and Max_Data = 0xFF */

}CanMsgTypeDef;

#if !defined(SIZE_RX_BUFFER)
#define SIZE_RX_BUFFER  16 // receive incoming ring buffer size
#endif

#if !defined(SIZE_TX_BUFFER)
#define SIZE_TX_BUFFER  8 // transmit ring buffer size

#define NUM_MAILBOXES   3 // architecture specific STM32F103Cx have 3 mailboxes

typedef struct RingbufferTypeDef {
    volatile uint16_t head;
    volatile uint16_t tail;
    uint16_t size;
    volatile CanMsgTypeDef *buffer;
} RingbufferTypeDef;

#else
#define MAXELEMENTS 5
typedef struct QueueRX
{
    int size;
    int front;
    int rear;
    CanMsgTypeDef elements[MAXELEMENTS];
} QueueRX;
#endif

class STM32F1_CAN
{
  protected:
   uint8_t numTxMailboxes;
   uint16_t sizeRxBuffer;
   uint16_t sizeTxBuffer;
  public:
    STM32F1_CAN ();
    static STM32F1_CAN& getInstance();
	void begin(bool UseAltPins);
    bool write (const CanMsgTypeDef &msg,bool wait_sent);
    bool write (const CanMsgTypeDef &msg,bool wait_sent, uint8_t n);
    bool read (CanMsgTypeDef &msg);

    // Before begin, you can define rx buffer size. Default is SIZE_RX_BUFFER. This does not have effect after begin.
    void setRxBufferSize(uint16_t size) { if (!isInitialized() ) sizeRxBuffer=size; }
    
    // Before begin, you can define global tx buffer size. Default is SIZE_TX_BUFFER. This does not have effect after begin.
    void setTxBufferSize(uint16_t size) { if (!isInitialized() ) sizeTxBuffer=size; }
    
    // You can define mailbox specific tx buffer size. This can be defined only once per mailbox.
    // As default prioritized messages will not be buffered. If you define buffer size for mail box, the messages will be
    // buffered to own buffer, if necessary.
    void setMailBoxTxBufferSize(uint8_t mbox, uint16_t size);
    bool addToRingBuffer (RingbufferTypeDef &ring, const CanMsgTypeDef &msg);
	bool removeFromRingBuffer (RingbufferTypeDef &ring, CanMsgTypeDef &msg);
  private:
//    inline uint8_t getFirstTxBox() { return getNumMailBoxes()-numTxMailboxes; }
//    inline uint8_t getLastTxBox() { return getNumMailBoxes()-1; }
    inline uint8_t getNumMailBoxes() { return NUM_MAILBOXES; }

    bool isInitialized() { return rx_buffer!=0; }
    void initRingBuffer (RingbufferTypeDef &ring, volatile CanMsgTypeDef *buffer, uint32_t size);
    void initializeBuffers(void);
    bool isRingBufferEmpty (RingbufferTypeDef &ring);
    uint32_t ringBufferCount (RingbufferTypeDef &ring);
  public:
    RingbufferTypeDef rxRing;
    RingbufferTypeDef txRing;
  private:
    volatile CanMsgTypeDef *rx_buffer;
    volatile CanMsgTypeDef *tx_buffer;
    RingbufferTypeDef * txRings[NUM_MAILBOXES];
};

//extern STM32F1_CAN Can0;

#endif
