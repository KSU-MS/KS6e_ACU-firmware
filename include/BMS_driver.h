

#ifndef BMS_DRIVER_H
#define BMS_DRIVER_H


#include <Arduino.h>


#define                    BMS_ID 0x7E3
#define        ThermistorToBMS_ID 0x9839F380 
#define ThermistorAddressClaim_ID 0x98EEFF80 //probably unnecessary
#define           BMS_Response_ID 0x7EB





typedef struct
{

    uint8_t temp[8];

    uint8_t avgTemp = 0;


} temperature;





#endif