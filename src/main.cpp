
#include <Arduino.h>
#include "FlexCAN_handle.h"


CAN_message_t rxMsg;

uint8_t temp1[8];
uint8_t temp2[8];


void setup()
{

    Serial.begin(9600);

    init_CAN();

}


void loop()
{

    if(ReadCAN(rxMsg))
    {

        if (rxMsg.id == 0xB0)
        {

            memcpy(temp1, rxMsg.buf, rxMsg.len);

        }
        else if (rxMsg.id == 0xB1)
        {

            memcpy(temp2, rxMsg.buf, rxMsg.len);

        }


    }

    delay(1000);

    for (int i = 0; i < rxMsg.len; i++)
    {

        Serial.print("Temp1 ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(temp1[i]);
        Serial.println();

    }

    for (int i = 0; i < rxMsg.len; i++)
    {

        Serial.print("Temp2 ");
        Serial.print(i);
        Serial.print(": ");
        Serial.print(temp2[i]);
        Serial.println();

    }

}

