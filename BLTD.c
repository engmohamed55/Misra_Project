/*
 * BLTD.c
 *
 * Created: 31/07/2015 12:35:22 ص
 *  Author: hossam
 */ 
#include "Basic_Types.h"
#include "UART_Drv.h"
#include "BLTD.h"

static u8 MemCmp(const u8  Src1[],const u8 Src2[],u16 CmpLength);

/***************************************************************************************************************/
void BLTD_SendInitCmd(void)
{	IsRespRecieved = 0U;
	UART_StartReception(RxBuffer,(u8)4,&RxcCallBackFun);
	BTCommandSend((u8*)"+INIT",(u8)5);
}
/***************************************************************************************************************/
void BLTD_SendInquireCmd(void)
{	IsRespRecieved = 0U;
	UART_StartReception(RxBuffer,(u8)4,&RxcCallBackFun);
	BTCommandSend((u8*)"+INQ",4U);
}
u8 BLTD_CheckForResponse(const u8 Response[],u16 RespLength)
{
	u8 RespStatus=0;
	u16 IsEqual=0;
	if(IsRespRecieved == 1U)
	{
		IsRespRecieved = 0U;
		IsEqual = MemCmp(Response,RxBuffer,(u16)RespLength);
		if(IsEqual == 0U)
		{
			RespStatus = BLTD_RESP_STATUS_OK;
			
		}
		else
		{
			RespStatus = BLTD_RESP_STATUS_NOK;
		}
		
	}
	else
	{

		RespStatus = BLTD_RESP_STATUS_NON;
	}
	return RespStatus;
}	
/***************************************************************************************************************/
void BLTD_StartWaitPairing(void)
{
	UART_StopRception();
	UART_StartReception(RxBuffer,(u8)4,&RxcCallBackFun);
	/*BTCommandSend(0,0);*/
	
}
/***************************************************************************************************************/	
void BLTD_SendMessage(const u8 Message[],u16 MsgLength)
{
	UART_TxBuffer(Message,MsgLength);
}	
/***************************************************************************************************************/
u8 BLTD_GetRecievedData(u8 Data[], u16 Length)
{
	u8 RespStatus2;
	u8 i;
	if(IsRespRecieved == 1U)
	{
		IsRespRecieved = 0U;
		RespStatus2 = BLTD_RESP_STATUS_OK;
		for( i = 0U; i< Length ; i++)
		{
			Data[i] = RxBuffer[i];
		}
	}
	else
	{
		RespStatus2 = BLTD_RESP_STATUS_NON;
	}		
		
	return RespStatus2;
}
/***************************************************************************************************************/
void BLTD_StartReceivingData(u8* DataBuffer,u16 BufferLength,CbkPfnType CbkFnPtr)
{
	UART_StartReception(DataBuffer,BufferLength,CbkFnPtr);
	
}
/***************************************************************************************************************/
u8 BLTD_CheckForData(u8* Data)
{
	u16 RxBytesNum;
	u8 IsReceived;
	RxBytesNum = UART_GetNumOfRxbytes();
	if(RxBytesNum > (u16)0)
	{
		IsReceived = (u8)0x01;
		*Data = RxBuffer[RxBytesNum+(u16)1];
		UART_StopRception();
		UART_StartReception(RxBuffer,(u8)100,&RxcCallBackFun);
	}
	else
	{
		IsReceived = (u8)0x00;
		*Data  = (u8)0;
	}
	return IsReceived;
}

/***************************************************************************************************************/	
void BLTD_SenTestCmd(void)
{
	UART_StartReception(RxBuffer,(u8)4,&RxcCallBackFun);
	
}
/***************************************************************************************************************/
static void BTCommandSend(const u8 Command[],u16 CommandLength)
	{
    static u8 BTCommandBuffer[100U];
		BTCommandBuffer[0] = (u8)'A';
		BTCommandBuffer[1] = (u8)'T';
		MemCpy2(&BTCommandBuffer[(u8)2],Command,CommandLength);
		BTCommandBuffer[CommandLength+(u8)2] = (u8)0x0d;
		BTCommandBuffer[CommandLength+(u8)3] = (u8)0x0a;
		UART_TxBuffer(BTCommandBuffer,CommandLength + (u8)4);
		
	}
/***************************************************************************************************************/
static  void MemCpy2(u8 Des[],const u8 Src[],u16 Length)
	{
	u16 i2;
	for(i2 = (u16)0 ; i2<Length ; i2++)
		{
		Des[i2] = Src[i2];
		}
	}
/***************************************************************************************************************/	
static void RxcCallBackFun(void)
{

	IsRespRecieved = (u8)1;
}
/***************************************************************************************************************/
static u8 MemCmp(const u8  Src1[],const u8 Src2[],u16 CmpLength)
	{
	u8 RetVal = 0;
	u16 i3;
	for(i3 = (u8)0 ;(i3 < CmpLength); i3++)
		{
		if( Src1[i3] != Src2[i3])
			{
			RetVal = (u8)1;
			break;
			}
		}
	return RetVal;
	}
/***************************************************************************************************************/
