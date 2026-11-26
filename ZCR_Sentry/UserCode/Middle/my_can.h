#ifndef __MYCAN_H__
#define __MYCAN_H__

#include "main.h"
#include "can.h"

typedef struct My_Can_rx My_Can_rx;
typedef void (*MyCanRxCallback_t)(const My_Can_rx *rx);

struct My_Can_rx
{
	uint8_t can_rx[8];
	CAN_RxHeaderTypeDef rx_header;
	MyCanRxCallback_t my_can_rx_callback;
};

void MycanInit(void);
const My_Can_rx* MyCanGetRx(void);
void MyCanSetRxCallback(MyCanRxCallback_t cb);
void MyCanSend(uint16_t id, uint8_t *tx_data);

#endif