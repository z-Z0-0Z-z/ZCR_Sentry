#include "my_can.h"

static My_Can_rx _my_can_rx;
uint32_t TxMailbox_number;

void MycanInit(void)
{
	CAN_FilterTypeDef filter_Init;
	filter_Init.FilterActivation=ENABLE;
	filter_Init.FilterBank=0;
	filter_Init.FilterFIFOAssignment=CAN_RX_FIFO0;
	filter_Init.FilterIdHigh=0x0000;
	filter_Init.FilterIdLow=0x0000;
	filter_Init.FilterMaskIdHigh=0x0000;
	filter_Init.FilterMaskIdLow=0x0000;
	filter_Init.FilterMode=CAN_FILTERMODE_IDMASK;
	filter_Init.FilterScale=CAN_FILTERSCALE_16BIT;
	filter_Init.SlaveStartFilterBank=14;
	
	HAL_CAN_ConfigFilter(&hcan1,&filter_Init);
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING); 
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if (hcan == &hcan1){
		HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(_my_can_rx.rx_header), _my_can_rx.can_rx);
		if (_my_can_rx.my_can_rx_callback != NULL) {
			_my_can_rx.my_can_rx_callback(&_my_can_rx);   /* hand the frame to the registered handler immediately */
		}
	}
}

void MyCanSetRxCallback(MyCanRxCallback_t cb)
{
	_my_can_rx.my_can_rx_callback = cb;
}

const My_Can_rx* MyCanGetRx(void)
{
	return &_my_can_rx;
}


void MyCanSend(uint16_t id,uint8_t *tx_data)
{
	CAN_TxHeaderTypeDef tx_header;
	tx_header.StdId=id;
	tx_header.ExtId=0x00;
	tx_header.RTR=CAN_RTR_DATA;
	tx_header.IDE=CAN_ID_STD;
	tx_header.DLC=8;
	HAL_CAN_AddTxMessage(&hcan1,&tx_header,tx_data,&TxMailbox_number);
}