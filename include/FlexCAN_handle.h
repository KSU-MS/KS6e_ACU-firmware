

#ifndef FLEXCAN_HANDLE_H
#define FLEXCAN_HANDLE_H

#include <FlexCAN_T4.h>


#define NUM_TX_MAILBOXES 6
#define NUM_RX_MAILBOXES 2


void init_CAN();
int ReadCAN(CAN_message_t &msg);


#endif