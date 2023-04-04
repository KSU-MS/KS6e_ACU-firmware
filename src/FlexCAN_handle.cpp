

#include "FlexCAN_handle.h"


FlexCAN_T4 <CAN2, RX_SIZE_256, TX_SIZE_16> ACU_;
FlexCAN_T4 <CAN1, RX_SIZE_256, TX_SIZE_16> CAN_1;


void init_CAN()
{

    ACU_.begin();
    ACU_.setBaudRate(500000);

    ACU_.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);

    for (int i = 0; i < NUM_RX_MAILBOXES; i++)
    {

        ACU_.setMB((FLEXCAN_MAILBOX)i, RX, STD);
        
    }

    for (int i = NUM_RX_MAILBOXES; i < (NUM_TX_MAILBOXES + NUM_RX_MAILBOXES); i++)
    {

        ACU_.setMB((FLEXCAN_MAILBOX)i, TX, STD);

    }

}


int ReadBatteryTemps(CAN_message_t &msg)
{
    
    int rxMSG = ACU_.read(msg);

    return rxMSG;

}


void WriteToBMS(uint32_t id, uint8_t buf[])
{

    CAN_message_t msg;

    msg.id = id;

    msg.flags.extended = true;


    msg.buf[0] = buf[6];
    msg.buf[1] = buf[7];

    CAN_1.write(msg);

}