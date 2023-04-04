
#include <Arduino.h>
#include "FlexCAN_handle.h"


CAN_message_t rxMsg;

uint8_t temp[8];


void setup()
{

    Serial.begin(9600);

    init_CAN();

}


void loop()
{

    if(ReadCAN(rxMsg))
    {

        memcpy(rxMsg.buf, temp, rxMsg.len);

    }

    delay(1000);

    for (int i = 0; i < rxMsg.len; i++)
    {

        Serial.print("Temp ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(temp[i]);
        Serial.println();

    }

}

