// STM23F103C8 - Displays all traffic found on either canbus port
// By Thibaut Viard/Wilfredo Molina/Collin Kidder 2013-2014
// jaume clarens 2018

// Required libraries
#include <STM32F1_CAN2.h>
HardwareSerial Serial2(USART2);
HardwareSerial Serial3(USART3); //CAN debug

void setup()
{
	Serial.begin(115200);
	Serial.println("hI it's 1\n\r");
	Serial3.begin(115200);
	Serial3.println("hI it's 3 \n\r");
	Serial2.begin(115200);
	Serial2.println("hI it's 2\n\r");
 //   Can0.begin(false); //Use pins (PA_11 and PA_12)
//    Can0.begin(true); //Use pins ( PB_8 and PB_9)

}

void printFrame(CanMsgTypeDef &frame) {
   Serial.print("ID: 0x");
   Serial.print(frame.id, HEX);
   Serial.print(" Len: ");
   Serial.print(frame.len);
   Serial.print(" Data: 0x");
   for (int count = 0; count < frame.len; count++) {
       Serial.print(frame.Data[count], HEX);
       Serial.print(" ");
   }
   Serial.print("\r\n");
}

void loop(){
	delay(1);
	return;
	CanMsgTypeDef incoming;

  if (Can0.available() > 0) {
	  Can0.read(incoming);
	printFrame(incoming);
  }
}


