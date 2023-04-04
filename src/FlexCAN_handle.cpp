

#include "FlexCAN_handle.h"


FlexCAN_T4 <CAN2, RX_SIZE_256, TX_SIZE_16> ACU_;


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


int ReadCAN(CAN_message_t &msg)
{
    
    int rxMSG = ACU_.read(msg);

    return rxMSG;

}