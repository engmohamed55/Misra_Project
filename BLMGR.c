#include "Basic_Types.h"
#include "BLTD.h"
#include "BLMGR.h"
#include "CRC.h"
#include "DIO.h"
#include "UART_Drv.h"
#include "BLMGR_CFG.h"
#include "delay.h"


/*********************************************************************************/
/*Local Symbols*/
/*********************************************************************************/
/*Paring States*/
#define PAIRING_STATE_IDLE                  ((unsigned char)0xff)
#define PAIRING_STATE_INITIALIZING          ((unsigned char)0x00)
#define PAIRING_STATE_WAIT_INIT_RESP        ((unsigned char)0x01)
#define PAIRING_STATE_INQUIRE               ((unsigned char)0x02)
#define PAIRING_STATE_WAIT_INQUIRE_RESP     ((unsigned char)0x03)
#define PAIRING_STATE_WAIT_PAIR_REQ         ((unsigned char)0x04)
#define PAIRING_STATE_CONNECTED_DONE        ((unsigned char)0x05)
#define PAIRING_STATE_FAILED                ((unsigned char)0x06)
#define PAIRING_STATE_START_WAIT_PAIR_REQ   ((unsigned char)0x07)
/*********************************************************************************/
/*Handshaking states*/
#define HANDSHAKE_STATE_IDLE             ((unsigned char)0xff)
#define HANDSHAKE_STATE_SEND_ID_FRAME    ((unsigned char)0x01)
#define HANDSHAKE_STATE_RECV_ID_FRAME    ((unsigned char)0x02)
#define HANDSHAKE_STATE_SEND_VAL_FRMAE   ((unsigned char)0x03)
#define HANDSHAKE_STATE_RECV_VAL_FRAME   ((unsigned char)0x04)
#define HANDSHAKE_STATE_SEND_ERR_FRAME   ((unsigned char)0x05)
#define HANDSHAKE_STATE_HANDSHAKING_DONE ((unsigned char)0x06)
#define HANDSHAKE_STATE_FAILED           ((unsigned char)0x07)
/*********************************************************************************/
/*Communication states*/
#define COMM_STATE_IDLE             ((unsigned char) 0xff)
#define COMM_STATE_SEND_DATA_FRAME  ((unsigned char) 0x01)
#define COMM_STATE_RECV_DATA_FRAME  ((unsigned char) 0x02)
#define COMM_STATE_SEND_ERROR_FRAME ((unsigned char) 0x03)
#define COMM_STATE_FAILED           ((unsigned char) 0x04)
/*********************************************************************************/
/*Bluetooth States*/
#define BLUETOOTH_STATE_STOPPED        ((unsigned char)0xff)
#define BLUETOOTH_STATE_DISCONNECTED   ((unsigned char)0x00)
#define BLUETOOTH_STATE_PAIRING        ((unsigned char)0x01)
#define BLUETOOTH_STATE_HANDSHAKING    ((unsigned char)0x02)
#define BLUETOOTH_STATE_COMMUNICATION  ((unsigned char)0x03)
/*********************************************************************************/
/*Lengths Configuration*/
#define ID_FRAME_LENGTH         ((unsigned char)18)
#define VAL_FRAME_LENGTH        ((unsigned char)18)
#define DATA_FRAME_LENGTH       ((unsigned char)18)
#define ERROR_FRAME_LENGTH      ((unsigned char)18)
#define MAX_DEV_NAME_LENGTH     ((unsigned char)6)
#define MAX_DATA_BUFFER_LENGTH  ((unsigned char)18)
/*********************************************************************************/
/*Timeout Counts Configuration*/
#define PAIRING_MAX_COUNTS      ((unsigned char)1)
#define HANDSHAKING_MAX_COUNTS  ((unsigned char)10)
#define COMM_MAX_COUNTS         ((unsigned char)10)
#define MAX_PAIRING_FAIL_REPT   ((unsigned char)10)
#define MAX_HANDSHAKE_FAIL_REP  ((unsigned char)5)
#define MAX_COMM_FAIL_REP       ((unsigned char)20)
#define MAX_DISCONNECTION_COUNT ((unsigned char) 2)
/*********************************************************************************/
/*Error States*/
#define ERRH_TIMEOUT_ERROR             ((unsigned char) 0x00)
#define ERRH_HANDSHAKE_ERROR           ((unsigned char)0x01)
#define ERRH_CHECKSUM_ERROR            ((unsigned char)0x03)
#define ERRH_INVALID_FRAME_ERROR       ((unsigned char)0x04)
#define ERRH_CRC_ERROR                 ((unsigned char)0x05)
#define ERRH_INVALID_RECEIVER_ERROR    ((unsigned char) 0x06)
#define ERRH_WRONG_FRAME_ERROR         ((unsigned char) 0x07)
#define ERRH_NO_ERROR                  ((unsigned char) 0xff)
/*********************************************************************************/
/*Buffer Indices*/
#define FRAME_HEADER_IDX   ((unsigned char)0x00)
#define FRAME_SENDER_IDX   ((unsigned char)0x02)
#define FRAME_RECVER_IDX   ((unsigned char)0x03)
#define FRAME_TYPE_IDX     ((unsigned char)0x04)
#define PARAM_LENGTH_IDX   ((unsigned char)0x05)
#define OS_TYPE_IDX        ((unsigned char)0x06)
#define DEV_TYPE_IDX       ((unsigned char) 0x07)
#define DEV_NAME_IDX       ((unsigned char) 0x08)
#define FRAME_CRC_IDX      ((unsigned char) 0x0E)
#define FRAME_CHECKSUM_IDX ((unsigned char)0x10)
#define FRAME_TAIL_IDX     ((unsigned char) 0x11)
#define FRAME_VAL_CRC_IDX  ((unsigned char) 0x06)
#define BATT_LEVEL_IDX     ((unsigned char) 0x06)
#define DIRECTION_IDX      ((unsigned char)0x06)
#define SPEED_DEGREE_IDX   ((unsigned char) 0x07)
#define ERROR_TYPE_IDX     ((unsigned char) 0x06)
/*********************************************************************************/
/*Default Values*/
#define TX_OS_TYPE       ((unsigned char)0xff)
#define TX_DEV_TYPE      ((unsigned char)0x04)
#define TX_FRAME_DEFUALT ((unsigned char)0x00)
#define TX_CRC_DEFAULT   ((unsigned char)0xff)
/*********************************************************************************/
/*Frame types*/
#define FRAME_TYPE_ID_FRAME    ((unsigned char)0x01)
#define FRAME_TYPE_VAL_FRAME   ((unsigned char)0x02)
#define FRAME_TYPE_DATA_FRAME  ((unsigned char)0x03)
#define FRAME_TYPE_ERROR_FRAME ((unsigned char)0x04)
/*********************************************************************************/
/*Error Types*/
#define ERROR_TYPE_RESEND_LAST_FRMAE     ((unsigned char)0x01)
#define ERROR_TYPE_START_NEW_HANDSHAKE   ((unsigned char)0x02)
#define ERROR_TYPE_UPDATE_UR_TRANSMITTER ((unsigned char)0x03)
/*********************************************************************************/
/*Private functions*/
/*********************************************************************************/
/*State machines*/
static void PairingInit(void);
static void PairingStateMachine(void);
static void ErrorHandlingStateMachine(void);
static void HandShakeInit(void);
static void HandShakingStateMachine(void);
static void CommunicationInit(void);
static void CommStateMachine(void);
static void DisconnectInit(void);
static void DisconnectStateMachine(void);
/*********************************************************************************/
/*Frames handlers*/
static void UpdateIdFrame(void);
static u8   CheckIdFrame(void);
static void UpdateValFrame(void);
static u8   CheckValFrame(void);
static void UpdateDataFrame(void);
static u8 CheckDataFrame(void);
static void UpdateErrorFrame(u8 ErrorType);
static void CheckErrorFrame(void);
/*********************************************************************************/
/*Utilities*/
static void RxBufferDnCallBackNotif(void);
static void MemCpy( u8* DesPtr, const u8* SrcPtr, u8 Length);
static void MemSet(u8* DesPtr, u8 ConstVal, u8 Length);
#if (defined(COMM_CINFIG) == defined(SLAVE_COMM))
static u8 MemCmp(const u8* Src1Ptr,const u8* Src2Ptr,u16 Length);
static u8 GetCrcKey(void);
#endif

static u8 CalculateCheksum(const u8* BufferPtr, u16 BufferLength);
static void BuzzerSound(void);
static u8 CheckCrc(void);
static void PowerBluetoothOn(void);
static void PowerBluetoothOff(void);
static void BuzzerInit(void);
static void PowerBlueToothInit(void);
static void BlueToothKeyInit(void);
static void InserBreakPoint(void);
/*********************************************************************************/
/*Static variables*/
/*********************************************************************************/
static u8  BLMGR_PairingState;
static u32 BLMGR_PairingTimeoutCounter;
static u8  BLMGR_HandShakeState;
static u8  BLMGR_HandShakeTimeOutCounter;
static u8  BLMGR_DataRxBuffer[MAX_DATA_BUFFER_LENGTH];
static u8  BLMGR_DataTxBuffer[MAX_DATA_BUFFER_LENGTH];
static u8  BLMGR_FrameReceivedNotificationFlag;
static u8  BLMGR_ErrorState;
static u8  BLMGR_RxOsType;
static u8  BLMGR_RxDeviceType;
static u8  BLMGR_RxDevicName[MAX_DEV_NAME_LENGTH];
static u8  BLMGR_TxDevicName[MAX_DEV_NAME_LENGTH];
static u8  BLMGR_RxDeviceNameLength;
static u8  BLMGR_TxDeviceNameLength;
static u8  BLMGR_TxFrameReceiver;
static u8  BLMGR_RxFrameSender;
static u32 BLMGR_CrcKey;
static u8  BLMGR_CommState;
static u8  BLMGR_CommTimeOutCounter;
static u8  BLMGR_TxBattLevel;
#if (defined(COMM_CINFIG) == defined(SLAVE_COMM))
static u8  BLMGR_TxDirection;
static u8  BLMGR_TxSpeedDegree;
static u8  BLMGR_RxBattLevel;
#endif
static u8  BLMGR_BluetoothState;
static u8  BLMGR_BluetoothStartRequest;
static u8  BLMGR_StopDeviceRequest;
static u8  BLMGR_PairingFailRepetionCount;
static u8  BLMGR_HandshakeFailRepCount;
static u8  BLMGR_CommFailReptCount;
static u8  BLMGR_ExpectedReceivedFrame;
static u8  BLMGR_DisconectionTimeOut;
static u8 testflag = 0;
static u8 BLMGR_DevicePaired;
/*********************************************************************************/
/*Global Services*/
/*********************************************************************************/
void BLMGR_StartDevice(void)
{
    BLMGR_BluetoothStartRequest = 1u;
}
/*********************************************************************************/
void BLMGR_Test(void)
{
    BuzzerSound();



}

/*********************************************************************************/
void BLMGR_BluetoothInit(void)
{
    /*Init UART*/
    UART_Init();
    /*Init State*/
    BLMGR_BluetoothState = BLUETOOTH_STATE_STOPPED;
    BLMGR_BluetoothStartRequest = 0u;
    BLMGR_StopDeviceRequest = 0u;
    BLMGR_DevicePaired = 0u;
    /*Init Pairing*/
    PairingInit();
    /*Init Handshaking*/
    HandShakeInit();
    /*Init Communication*/
    CommunicationInit();
    /*Disconnection Init*/
    DisconnectInit();
    /*Init Buzzer*/
    BuzzerInit();
    /*Init Bluetooth Power Control*/
    PowerBlueToothInit();
    /*Init Key*/
    BlueToothKeyInit();

}
/*********************************************************************************/
void BLMGR_BluetoothStateMachine(void)
{
    switch(BLMGR_BluetoothState)
    {
    case BLUETOOTH_STATE_STOPPED:
    {
        /*Check if application need to start bluetooth*/
        if(BLMGR_BluetoothStartRequest == 1u)
        {
            /*Power On the module*/
            PowerBluetoothOn();
            PairingInit();
            BLMGR_BluetoothState = BLUETOOTH_STATE_PAIRING;

        }
    }
    break;

    case BLUETOOTH_STATE_PAIRING:
    {
        PairingStateMachine();
        if(BLMGR_PairingState == PAIRING_STATE_CONNECTED_DONE)
        {
            /*BLMGR_Test();*/
            /*Pairing succeeded, start handshaking*/
            HandShakeInit();
            BLMGR_BluetoothState = BLUETOOTH_STATE_HANDSHAKING;
        }
        else if(BLMGR_PairingState == PAIRING_STATE_FAILED)
        {
            /*Pairing failed, disconnect the module*/
            PairingInit();
            BLMGR_BluetoothState = BLUETOOTH_STATE_DISCONNECTED;
        }
        else
        {
            /*pairing is in progress*/
        }
    }
    break;
    case BLUETOOTH_STATE_HANDSHAKING:
    {
        HandShakingStateMachine();
        if(BLMGR_HandShakeState == HANDSHAKE_STATE_HANDSHAKING_DONE)
        {
            /*Handshake succeeded, start communication*/
            CommunicationInit();
            BLMGR_BluetoothState = BLUETOOTH_STATE_COMMUNICATION;
            BuzzerSound();
        }
        else if (BLMGR_HandShakeState == HANDSHAKE_STATE_FAILED)
        {
            /*handshake failed, disconnect the module*/
            HandShakeInit();
            BLMGR_BluetoothState = BLUETOOTH_STATE_DISCONNECTED;
        }
        else
        {
            /*handshake still in progress*/
        }
    }
    break;
    case BLUETOOTH_STATE_COMMUNICATION:
    {
        CommStateMachine();
        if(BLMGR_CommState == COMM_STATE_FAILED)
        {
            /*Disconnect the module*/
            CommunicationInit();
            BLMGR_BluetoothState = BLUETOOTH_STATE_DISCONNECTED;
        }
        else
        {
            /*Communication is in progress*/

        }
    }
    break;

    case BLUETOOTH_STATE_DISCONNECTED:
    {
        DisconnectStateMachine();
        /*Check if application need to start bluetooth*/
        if(BLMGR_BluetoothStartRequest == 1u)
        {
            /*Power On the module*/
            /*PowerBluetoothOn();*/
            PairingInit();
            /*PowerBluetoothOff();*/
            /*InserBreakPoint();*/
            BLMGR_PairingState = PAIRING_STATE_WAIT_PAIR_REQ;
            if(BLMGR_DisconectionTimeOut > MAX_DISCONNECTION_COUNT)
            {
                BLMGR_BluetoothState = BLUETOOTH_STATE_PAIRING;
                BLMGR_DisconectionTimeOut = 0u;

            }
            else
            {
                /*	BuzzerSound();*/
                BLMGR_DisconectionTimeOut ++;
            }

        }
        else if (BLMGR_StopDeviceRequest == 1u)
        {
            /*PowerBluetoothOff();*/
            DisconnectInit();
            BLMGR_BluetoothState = BLUETOOTH_STATE_STOPPED;
        }
        else
        {
            /*Still disconnected*/
        }
    }
    break;
    default:
    {

    }
    break;
    }
}
/*********************************************************************************/
/*Private Funcions definitions*/
/*********************************************************************************/
static void PairingInit(void)
{
    BLMGR_PairingState = PAIRING_STATE_IDLE;
    BLMGR_PairingTimeoutCounter = 0u;
    BLMGR_PairingFailRepetionCount = 0u;
    /*BLMGR_DevicePaired = 0;*/
}
/*********************************************************************************/
static void HandShakeInit(void)
{
    BLMGR_HandShakeState = HANDSHAKE_STATE_IDLE;
    BLMGR_PairingTimeoutCounter = 0u;
    BLMGR_FrameReceivedNotificationFlag = 0u;
    BLMGR_HandshakeFailRepCount = 0u;
}
/*********************************************************************************/
static void CommunicationInit(void)
{
    BLMGR_CommState = COMM_STATE_IDLE;
    BLMGR_CommTimeOutCounter = 0u;
    BLMGR_CommFailReptCount = 0u;
}
/*********************************************************************************/

static void PairingStateMachine(void)
{
    u8 ResponseState;

    if(BLMGR_DevicePaired == 1u)
    {
        /*InserBreakPoint();*/
        BLMGR_PairingState = PAIRING_STATE_START_WAIT_PAIR_REQ;
        BLMGR_DevicePaired = 0u;
    }
    switch(BLMGR_PairingState)
    {
    case PAIRING_STATE_IDLE:
    {
        /*wait for 1 second for stabilization*/
        if(BLMGR_PairingTimeoutCounter > PAIRING_MAX_COUNTS)
        {
            BLMGR_PairingTimeoutCounter = 0u;
            /*go to the init state*/
            BLMGR_PairingFailRepetionCount = 0u;
            BLMGR_PairingState = PAIRING_STATE_INITIALIZING;


        }
        else
        {
            BLMGR_PairingTimeoutCounter ++;
        }
    }
    break;

    case PAIRING_STATE_INITIALIZING:
    {
        /*send init Command*/
        BLTD_SendInitCmd();
        /*Go to next state to read response*/
        BLMGR_PairingState = PAIRING_STATE_WAIT_INIT_RESP;
    }
    break;

    case PAIRING_STATE_WAIT_INIT_RESP:
    {
        char RespArray[4];
        RespArray[0] = (char)'O';
        RespArray[1] = (char)'K';
        RespArray[2] =(char)0x0d;
        RespArray[3] =(char) 0x0a;

        ResponseState = BLTD_CheckForResponse(RespArray,4u);
        switch(ResponseState)
        {
        case BLTD_RESP_STATUS_OK:
            BLMGR_PairingFailRepetionCount = 0u;
            BLMGR_PairingTimeoutCounter = 0u;
            /*Respnse recieved and go to send the inquire request*/
            BLMGR_PairingState = PAIRING_STATE_INQUIRE;

            break;

        case BLTD_RESP_STATUS_NOK:

            if(BLMGR_PairingFailRepetionCount <= MAX_PAIRING_FAIL_REPT)
            {
                /*response received not ok so re send the command again*/
                BLMGR_PairingState = PAIRING_STATE_INITIALIZING;
                BLMGR_PairingFailRepetionCount ++;

            }
            else
            {
                BLMGR_PairingFailRepetionCount = 0u;
                BLMGR_PairingState = PAIRING_STATE_INQUIRE;
            }

            break;

        case BLTD_RESP_STATUS_NON:

            /*response not received and wait until timeout*/
            BLMGR_PairingTimeoutCounter ++;
            if(BLMGR_PairingTimeoutCounter > PAIRING_MAX_COUNTS)
            {
                if(BLMGR_PairingFailRepetionCount <= MAX_PAIRING_FAIL_REPT)
                {
                    BLMGR_PairingFailRepetionCount ++;
                    BLMGR_PairingTimeoutCounter = 0u;
                    BLMGR_PairingState = PAIRING_STATE_INITIALIZING;
                }
                else
                {
                    BLMGR_PairingFailRepetionCount = 0u;
                    BLMGR_PairingState = PAIRING_STATE_FAILED;
                }
            }
            else
            {
                BLMGR_PairingTimeoutCounter ++;
                BLMGR_PairingState = PAIRING_STATE_WAIT_INIT_RESP;
            }
            break;
        default:
            break;
        }
    }
    break;
    case PAIRING_STATE_INQUIRE:
    {

        /*Send Inquire Command*/
        BLTD_SendInquireCmd();
        /*wait for the Inquire Response*/
        BLMGR_PairingState = PAIRING_STATE_WAIT_INQUIRE_RESP;
    }
    break;
    case PAIRING_STATE_WAIT_INQUIRE_RESP:
    {
        char RespArray1[4];
        RespArray1[0] = (char)'O';
        RespArray1[1] = (char)'K';
        RespArray1[2] =(char)0x0d;
        RespArray1[3] =(char) 0x0a;
        ResponseState = BLTD_CheckForResponse(RespArray1,4u);
        switch(ResponseState)
        {
        case BLTD_RESP_STATUS_OK:
            BLMGR_PairingFailRepetionCount = 0u;
            BLMGR_PairingTimeoutCounter = 0u;
            /*Respnse recieved and go to send the inquire request*/
            BLMGR_PairingState = PAIRING_STATE_START_WAIT_PAIR_REQ;

            break;

        case BLTD_RESP_STATUS_NOK:
            if(BLMGR_PairingFailRepetionCount <= MAX_PAIRING_FAIL_REPT)
            {
                BLMGR_PairingFailRepetionCount ++;
                /*response received not ok so re send the command again*/
                BLMGR_PairingState = PAIRING_STATE_WAIT_INQUIRE_RESP;
            }
            else
            {
                BLMGR_PairingFailRepetionCount = 0u;
                BLMGR_PairingState = PAIRING_STATE_INITIALIZING;
            }
            break;

        case BLTD_RESP_STATUS_NON:
            /*response not received and wait until timeout*/
            BLMGR_PairingTimeoutCounter ++;
            if(BLMGR_PairingTimeoutCounter > PAIRING_MAX_COUNTS)
            {
                if(BLMGR_PairingFailRepetionCount <= MAX_PAIRING_FAIL_REPT)
                {
                    BLMGR_PairingFailRepetionCount ++;
                    BLMGR_PairingTimeoutCounter = 0u;
                    BLMGR_PairingState = PAIRING_STATE_INQUIRE;
                }
                else
                {
                    BLMGR_PairingFailRepetionCount = 0u;
                    BLMGR_PairingState = PAIRING_STATE_FAILED;
                }
            }
            else
            {
                BLMGR_PairingTimeoutCounter ++;
                BLMGR_PairingState = PAIRING_STATE_WAIT_INQUIRE_RESP;
            }
            break;
        default:
            break;
        }
    }
    break;
    case PAIRING_STATE_START_WAIT_PAIR_REQ:
        BLTD_StartWaitPairing();
        BuzzerSound();
        BLMGR_PairingState = PAIRING_STATE_WAIT_PAIR_REQ;
        break;
    case PAIRING_STATE_WAIT_PAIR_REQ:
    {
        char RespArray2[4u];
        RespArray2[0] = (char)'O';
        RespArray2[1] = (char)'K';
        RespArray2[2] = (char)0x0d;
        RespArray2[3] = (char) 0x0a;
        ResponseState = BLTD_CheckForResponse(RespArray2,4u);
        switch(ResponseState)
        {
        case BLTD_RESP_STATUS_OK:
            BLMGR_PairingFailRepetionCount = 0u;
            BLMGR_PairingTimeoutCounter = 0u;
            /*Respnse recieved and go to send the inquire request*/
            BLMGR_PairingState = PAIRING_STATE_CONNECTED_DONE;
            /*BuzzerSound();*/
            BLMGR_DevicePaired = 1u;
            break;

        case BLTD_RESP_STATUS_NOK:
            if(BLMGR_PairingFailRepetionCount <= MAX_PAIRING_FAIL_REPT)
            {
                BLMGR_PairingFailRepetionCount ++;
                /*response received not ok so re send the command again*/
                BLMGR_PairingState = PAIRING_STATE_INQUIRE;
            }
            else
            {
                BLMGR_PairingFailRepetionCount = 0u;
                BLMGR_PairingState = PAIRING_STATE_FAILED;
            }
            break;

        case BLTD_RESP_STATUS_NON:
            /*response not received and wait until timeout*/
            BLMGR_PairingState = PAIRING_STATE_WAIT_PAIR_REQ;
            /*	BuzzerSound();*/
            break;
        default:
            break;
        }
    }
    break;
    default:
        break;
    }
}
/*********************************************************************************/
static void HandShakingStateMachine(void)
{
    u8 IsFrameValid;
    switch (BLMGR_HandShakeState)
    {
    case HANDSHAKE_STATE_IDLE:
    {
        /*Check for the Device Comm Mode*/
#if (defined(COMM_CINFIG) == defined(MSTER_COMM))
        /* the device will master the communication*/
        BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_ID_FRAME;
#elif(COMM_CINFIG == SLAVE_COMM)
        /*the Device will be mastered by the other device*/
        BLTD_StartReceivingData(BLMGR_DataRxBuffer,ID_FRAME_LENGTH,RxBufferDnCallBackNotif);
        BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_ID_FRAME;
#else
        /*Wrong Config, State in Idle*/
        /*To do: Managing dev Errors*/
#endif
    }
    break;
    case HANDSHAKE_STATE_SEND_ID_FRAME:
    {
        /*Update the ID frame by  loading tx data buffer*/
        UpdateIdFrame();
#if(defined(COMM_CINFIG) == defined(MSTER_COMM))
        /*Start Receiving the slave response*/
        BLTD_StartReceivingData(BLMGR_DataRxBuffer,ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
        BLMGR_ExpectedReceivedFrame = FRAME_TYPE_ID_FRAME;
        BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_ID_FRAME;
#elif(COMM_CINFIG == SLAVE_COMM)
        /*Start receiving Validation frame*/
        BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,RxBufferDnCallBackNotif);
        BLMGR_ExpectedReceivedFrame = FRAME_TYPE_VAL_FRAME;
        BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_VAL_FRAME;
#else
        /*Wrong Config, State in Idle*/
        /*To do: Managing dev Errors*/
#endif
        /*Send the ID Frame*/
        BLTD_SendMessage(BLMGR_DataTxBuffer,ID_FRAME_LENGTH);
    }
    break;
    case HANDSHAKE_STATE_RECV_ID_FRAME:
    {
        /*Check that a frame was received*/
        if(BLMGR_FrameReceivedNotificationFlag == 1u)
        {
            BLMGR_FrameReceivedNotificationFlag = 0u;
            BLMGR_HandShakeTimeOutCounter = 0u;
            IsFrameValid = CheckIdFrame();
            if(IsFrameValid == 1u)
            {
                BLMGR_HandshakeFailRepCount = 0u;
                /*Frame is valid*/
#if(defined(COMM_CINFIG) == defined(MSTER_COMM))
                /*Send the Validation frame*/
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLMGR_HandShakeState = HANDSHAKE_STATE_HANDSHAKING_DONE;
#elif(COMM_CINFIG == SLAVE_COMM)
                /*Start receiving validation frame*/
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,RxBufferDnCallBackNotif);
                BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_VAL_FRAME;
#else
                /*Wrong Config, State in Idle*/
                /*To do: Managing dev Errors*/
#endif
            }
            else
            {
                /*Frame is invalid*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_ERR_FRAME;
            }
        }
        else
        {
            /*No frame Received, Check Timeout*/
            if(BLMGR_HandShakeTimeOutCounter > HANDSHAKING_MAX_COUNTS)
            {
                /*Handle Timeout Error*/
                BLMGR_HandShakeTimeOutCounter = 0u;
                BLMGR_ErrorState = ERRH_TIMEOUT_ERROR;
                BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_ERR_FRAME;
            }
            else
            {
                BLMGR_HandShakeTimeOutCounter ++;
            }
        }
    }
    break;
    case HANDSHAKE_STATE_SEND_VAL_FRMAE:
    {
        /*Prepare Validation frame*/
        UpdateValFrame();
        /*Sending Validation frame*/
        BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
        BLTD_SendMessage(BLMGR_DataTxBuffer,VAL_FRAME_LENGTH);
#if (defined(COMM_CINFIG) == defined(MSTER_COMM))
        BLMGR_ExpectedReceivedFrame = FRAME_TYPE_VAL_FRAME;
        BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_VAL_FRAME;
#elif(COMM_CINFIG == SLAVE_COMM)
        BLMGR_HandShakeState = HANDSHAKE_STATE_HANDSHAKING_DONE;
#else
        /*Wrong Config, State in Idle*/
        /*To do: Managing dev Errors*/
#endif
    }
    break;
    case HANDSHAKE_STATE_RECV_VAL_FRAME:
    {
        /*Check that a frame was received*/
        if(BLMGR_FrameReceivedNotificationFlag == 1u)
        {
            BLMGR_FrameReceivedNotificationFlag = 0u;
            BLMGR_HandShakeTimeOutCounter = 0u;
            IsFrameValid = CheckValFrame();
            if(IsFrameValid == 1u)
            {
                BLMGR_HandshakeFailRepCount = 0u;
#if (defined(COMM_CINFIG) == defined(MSTER_COMM))
                /*Master the Communication phase*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_HANDSHAKING_DONE;
#elif(COMM_CINFIG == SLAVE_COMM)
                BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_VAL_FRMAE;
                /*Start Receiving validation frame*/
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,RxBufferDnCallBackNotif);
#else
                /*Wrong Config, State in Idle*/
                /*To do: Managing dev Errors*/
#endif
            }
            else
            {
                /*Handle validation error*/
                BLMGR_ErrorState = ERRH_HANDSHAKE_ERROR;
                BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_ERR_FRAME;
            }
        }
        else
        {
            /*No frame Received, Check Timeout*/
            if(BLMGR_HandShakeTimeOutCounter > HANDSHAKING_MAX_COUNTS)
            {
                /*Handle Timeout Error*/
                BLMGR_HandShakeTimeOutCounter = 0u;
                BLMGR_ErrorState = ERRH_TIMEOUT_ERROR;
                BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_ERR_FRAME;
            }
            else
            {
                BLMGR_HandShakeTimeOutCounter ++;
            }
        }
    }
    break;
    case HANDSHAKE_STATE_SEND_ERR_FRAME:
    {
        ErrorHandlingStateMachine();
    }
    break;
    default:
        break;
    }
}
/*********************************************************************************/
static void CommStateMachine(void)
{
    u8 IsFrameValid1;
    switch (BLMGR_CommState)
    {
    case COMM_STATE_IDLE:
    {
#if(defined(COMM_CINFIG) == defined(MSTER_COMM))
        /*Start Sending Data frame*/
        BLMGR_CommState = COMM_STATE_SEND_DATA_FRAME;
#elif(COMM_CINFIG == SLAVE_COMM)
        /*Start Receiving data frame*/
        BLMGR_ExpectedReceivedFrame = FRAME_TYPE_DATA_FRAME;
        BLMGR_CommState = COMM_STATE_RECV_DATA_FRAME;
        BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,RxBufferDnCallBackNotif);
#else
        /*Wrong Config, State in Idle*/
        /*To do: Managing dev Errors*/
#endif
    }
    break;
    case COMM_STATE_SEND_DATA_FRAME:
    {
        /*Update Data Frame*/
        UpdateDataFrame();
        /*Start Receiving data frame*/
        BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
        BLMGR_ExpectedReceivedFrame = FRAME_TYPE_DATA_FRAME;
        BLMGR_CommState = COMM_STATE_RECV_DATA_FRAME;
        /*Send the Data Frame*/
        BLTD_SendMessage(BLMGR_DataTxBuffer,DATA_FRAME_LENGTH);
    }
    break;
    case COMM_STATE_RECV_DATA_FRAME:
    {
        /*Check that a frame was received*/
        if(BLMGR_FrameReceivedNotificationFlag == 1u)
        {
            BLMGR_FrameReceivedNotificationFlag = 0u;
            BLMGR_HandShakeTimeOutCounter = 0u;
            BLMGR_CommTimeOutCounter = 0u;
            /*Check Received data frame*/
            IsFrameValid1 = CheckDataFrame();
            if(IsFrameValid1 == 1u)
            {
                BLMGR_CommFailReptCount = 0u;
                BLMGR_CommState = COMM_STATE_SEND_DATA_FRAME;
            }
            else
            {
                BuzzerSound();
                BLMGR_CommFailReptCount = 0u;
                /*Handshaking failed*/
                BLMGR_CommState = COMM_STATE_FAILED;
            }
        }
        else
        {

            if(BLMGR_CommTimeOutCounter > COMM_MAX_COUNTS)
            {

                /*InserBreakPoint();*/
                /*Handle Timeout Error*/
                BLMGR_CommTimeOutCounter = 0u;
                BLMGR_ErrorState = ERRH_TIMEOUT_ERROR;
                BLMGR_CommState = COMM_STATE_SEND_ERROR_FRAME;
            }
            else
            {
                BLMGR_CommTimeOutCounter ++;
            }
        }
    }
    break;
    case COMM_STATE_SEND_ERROR_FRAME:
    {

        ErrorHandlingStateMachine();
    }
    break;
    default:
        break;
    }

}

/*********************************************************************************/
static void BuzzerSound(void)
{
    u8 LoopIndex;
    for(LoopIndex = 0u; LoopIndex < 2u ; LoopIndex ++)
    {
        DIO_WritePort((u8)BuzzerConfig.Portname,(u8)BUZEER_ON,(u8)BuzzerConfig.PortMask1);
        _delay_ms(25u);
        DIO_WritePort((u8)BuzzerConfig.Portname,(u8)BUZEER_ON,(u8)BuzzerConfig.PortMask1);
        _delay_ms(25u);

    }

}
/*********************************************************************************/
static void RxBufferDnCallBackNotif(void)
{

    BLMGR_FrameReceivedNotificationFlag = 1u;

}
/*********************************************************************************/
void BLMGR_SetReceiver(u8 Receiver)
{
    BLMGR_TxFrameReceiver = Receiver;
}
/*********************************************************************************/
void BLMGR_SetDeviceName(const u8 DeviceName[],u8 DeviceNameLength)
{
    MemCpy(BLMGR_TxDevicName,DeviceName,DeviceNameLength);
    BLMGR_TxDeviceNameLength = DeviceNameLength;
}
/*********************************************************************************/

static void UpdateIdFrame(void)
{
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],TX_FRAME_DEFUALT,MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],0xaau,2u);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = FRAME_TYPE_ID_FRAME;
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = (((unsigned char)2) + BLMGR_TxDeviceNameLength);
    /*Update Os Type*/
    BLMGR_DataTxBuffer[OS_TYPE_IDX] = TX_OS_TYPE;
    /*Update Device Type*/
    BLMGR_DataTxBuffer[DEV_TYPE_IDX] = TX_DEV_TYPE;
    /*Update Device Name*/
    MemCpy(&BLMGR_DataTxBuffer[DEV_NAME_IDX],BLMGR_TxDevicName,BLMGR_TxDeviceNameLength);
    /*update Default CRC*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_CRC_IDX],TX_CRC_DEFAULT,2u);
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,FRAME_CHECKSUM_IDX -1u);
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],0x55u,1u);
}
/*********************************************************************************/
static u8 CheckIdFrame(void)
{
    u8 IsFrameValid_1;
    u8 TempVar;
    /* Perform a Checksum on the frame*/
    TempVar = CalculateCheksum(BLMGR_DataRxBuffer,FRAME_CHECKSUM_IDX -1u);

    if (TempVar == BLMGR_DataRxBuffer[FRAME_CHECKSUM_IDX])
    {

        /*Perform Start and end of frame validation*/
        if((BLMGR_DataRxBuffer[FRAME_HEADER_IDX] == 0xaau) && (BLMGR_DataRxBuffer[FRAME_HEADER_IDX + 1u] == 0xaau) && (BLMGR_DataRxBuffer[FRAME_TAIL_IDX] == 0x55u))
        {

            /*Validate Frame Type*/
            if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == FRAME_TYPE_ID_FRAME)
            {

                /* Check CRC*/
                if((BLMGR_DataRxBuffer[FRAME_CRC_IDX] == TX_CRC_DEFAULT) &&
                        (BLMGR_DataRxBuffer[FRAME_CRC_IDX] == TX_CRC_DEFAULT))
                {
                    /*Validate Frame Receiver*/
                    if(BLMGR_DataRxBuffer[FRAME_RECVER_IDX] == DEVICE_ROLE)
                    {
                        /*Validate Device Name*/
                        BLMGR_RxDeviceNameLength = (unsigned char)8 - BLMGR_DataRxBuffer[PARAM_LENGTH_IDX];
                        if(BLMGR_RxDeviceNameLength <= MAX_DEV_NAME_LENGTH)
                        {

                            /*Update received paramters*/
                            /*Update Frame sender*/
                            BLMGR_RxFrameSender = BLMGR_DataRxBuffer[FRAME_SENDER_IDX];
                            /*Update OS Type*/
                            BLMGR_RxOsType = BLMGR_DataRxBuffer[OS_TYPE_IDX];
                            /*Update Device Type*/
                            BLMGR_RxDeviceType = BLMGR_DataRxBuffer[DEV_TYPE_IDX];
                            /*Update Device Name*/
                            MemCpy(BLMGR_RxDevicName,&BLMGR_DataRxBuffer[DEV_NAME_IDX],BLMGR_RxDeviceNameLength);
                            BLMGR_ErrorState = ERRH_NO_ERROR;
                            IsFrameValid_1 = 1u;
                        }
                        else
                        {
                            /*Invalid Frame receiver*/
                            BLMGR_ErrorState = ERRH_INVALID_FRAME_ERROR;
                            IsFrameValid_1 = 0u;
                        }
                    }
                    else
                    {
                        /*Invalid Frame receiver*/
                        BLMGR_ErrorState = ERRH_INVALID_RECEIVER_ERROR;
                        IsFrameValid_1 = 0u;
                    }
                }
                else
                {
                    /*Crc Error Found*/
                    BLMGR_ErrorState = ERRH_CRC_ERROR;
                    IsFrameValid_1 = 0u;
                }
            }
            else
            {
                /*Invalid Frame Type*/
                BLMGR_ErrorState = ERRH_WRONG_FRAME_ERROR;
                IsFrameValid_1 = 0u;
            }
        }
        else
        {
            /*Invalid Frame Detected*/
            BLMGR_ErrorState = ERRH_INVALID_FRAME_ERROR;
            IsFrameValid_1 = 0u;
        }
    }
    else
    {
        /*Checksum error*/
        BLMGR_ErrorState = ERRH_CHECKSUM_ERROR;
        IsFrameValid_1 = 0u;
    }
    return IsFrameValid_1;
}
/*********************************************************************************/
static void UpdateValFrame(void)
{
    u16 Crc=0;
    u32 CrcKey=0;
    static u8 TempBuffer[MAX_DATA_BUFFER_LENGTH];
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],TX_FRAME_DEFUALT,MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],(unsigned char)0xaa,(unsigned char)2);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = FRAME_TYPE_VAL_FRAME;
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = (unsigned char)2;
#if(defined(COMM_CINFIG) == defined(MSTER_COMM))
    /* Start Generating the Key for CRC*/
    SECR_CrcPolynomialGenerate(&CrcKey,(unsigned char)16);
    BLMGR_CrcKey = CrcKey;
#endif
    /*Calculate CRC*/
    /*Prepare Data*/
    TempBuffer[0x00] = BLMGR_RxOsType;
    TempBuffer[0x01] = BLMGR_RxDeviceType;
    MemCpy(&TempBuffer[(unsigned char)0x02],BLMGR_RxDevicName,BLMGR_RxDeviceNameLength);
    SECR_GnerateCrc(TempBuffer,(u16)BLMGR_RxDeviceNameLength + (u16)2, &Crc,BLMGR_CrcKey);
    /*Update Crc*/
    BLMGR_DataTxBuffer[FRAME_VAL_CRC_IDX] = (u8)Crc;
    BLMGR_DataTxBuffer[FRAME_VAL_CRC_IDX + 1u] = (u8)(Crc >> 8u);
    /*update Default CRC*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_CRC_IDX],TX_CRC_DEFAULT,2u);
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,FRAME_CHECKSUM_IDX -1u);
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],0x55u,1u);
}
/*********************************************************************************/
static u8 CheckValFrame(void)
{
    u8 IsFrameValid_2=0;
    u8 TempVar_2=0;
    /* Perform a Checksum on the frame*/
    TempVar_2 = CalculateCheksum(BLMGR_DataRxBuffer,FRAME_CHECKSUM_IDX -1u);
    if (TempVar_2 == BLMGR_DataRxBuffer[FRAME_CHECKSUM_IDX])
    {
        /*Perform Start and end of frame validation*/
        if((BLMGR_DataRxBuffer[FRAME_HEADER_IDX] == 0xaau) &&\
                (BLMGR_DataRxBuffer[FRAME_HEADER_IDX + 1u] == 0xaau) &&\
                (BLMGR_DataRxBuffer[FRAME_TAIL_IDX] == 0x55u))
        {
            /*Validate Frame Type*/
            if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == FRAME_TYPE_VAL_FRAME)
            {
                /* Check CRC*/
                if((BLMGR_DataRxBuffer[FRAME_CRC_IDX] == TX_CRC_DEFAULT) &&
                        (BLMGR_DataRxBuffer[FRAME_CRC_IDX] == TX_CRC_DEFAULT))
                {
                    /*Validate Frame Receiver*/
                    if(BLMGR_DataRxBuffer[FRAME_RECVER_IDX] == DEVICE_ROLE)
                    {
#if(defined (COMM_CINFIG) == defined (MSTER_COMM))
                        /*Validate CRC */
                        IsFrameValid_2 = CheckCrc();


#elif(COMM_CINFIG == SLAVE_COMM)
                        /*Get the Crc Key*/
                        IsFrameValid_2 = GetCrcKey();
#endif
                    }
                    else
                    {
                        /*Invalid Frame receiver*/
                        BLMGR_ErrorState = ERRH_INVALID_RECEIVER_ERROR;
                        IsFrameValid_2 = 0u;
                    }

                }
                else
                {
                    /*Crc Error Found*/
                    BLMGR_ErrorState = ERRH_CRC_ERROR;
                    IsFrameValid_2 = 0u;
                }
            }
            else
            {
                /*Invalid Frame Type*/
                BLMGR_ErrorState = ERRH_WRONG_FRAME_ERROR;
                IsFrameValid_2 = 0u;
            }
        }
        else
        {
            /*Invalid Frame Detected*/
            BLMGR_ErrorState = ERRH_INVALID_FRAME_ERROR;
            IsFrameValid_2 = 0u;
        }
    }
    else
    {
        /*Checksum error*/
        BLMGR_ErrorState = ERRH_CHECKSUM_ERROR;
        IsFrameValid_2 = 0u;
    }
    return IsFrameValid_2;
}
/*********************************************************************************/
void BLMGR_SetBattLevel(u8 BattLevel)
{
    BLMGR_TxBattLevel= BattLevel;
}

static void UpdateDataFrame(void)
{
    static u8 TempBuffer_1[MAX_DATA_BUFFER_LENGTH];
    u16 Crc_1=0;
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],TX_FRAME_DEFUALT,MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],0xaau,2u);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = FRAME_TYPE_DATA_FRAME;
#if(defined (COMM_CINFIG) == defined(MSTER_COMM))
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = 1u;
    /*Set Batterly level*/
    BLMGR_DataTxBuffer[BATT_LEVEL_IDX] = BLMGR_TxBattLevel;
    /*Calculate CRC*/
    MemCpy(TempBuffer_1,&BLMGR_DataTxBuffer[BATT_LEVEL_IDX],1u);
    SECR_GnerateCrc(TempBuffer_1,1u, &Crc_1,BLMGR_CrcKey);
#elif(defined (COMM_CINFIG) == defined (SLAVE_COMM))
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = 2;
    /*Set Direction*/
    BLMGR_DataTxBuffer[DIRECTION_IDX]= BLMGR_TxDirection;
    /*Set Speed degree*/
    BLMGR_DataTxBuffer[SPEED_DEGREE_IDX]= BLMGR_TxSpeedDegree;
    /*Calculate CRC*/
    MemCpy(TempBuffer_1,&BLMGR_DataTxBuffer[DIRECTION_IDX],2);
    SECR_GnerateCrc(TempBuffer_1,2, &Crc_1,BLMGR_CrcKey);
#else
    /*Wrong Config, State in Idle*/
    /*To do: Managing dev Errors*/
#endif
    /*Update Crc*/
    BLMGR_DataTxBuffer[FRAME_CRC_IDX] = (u8)Crc_1;
    BLMGR_DataTxBuffer[FRAME_CRC_IDX + 1u] = (u8)(Crc_1 >> 8u);
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,FRAME_CHECKSUM_IDX -1u);
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],0x55u,1u);
}
/*********************************************************************************/
static u8 CheckDataFrame(void)
{
   static u8  BLMGR_RxSpeedDegree=0;
    static u8  BLMGR_RxDirection=0;
    static u8 TempBuffer_2[MAX_DATA_BUFFER_LENGTH];
    u8 IsFrameValid_3=0;
    u8 TempVar_3=0;
    u16 GenCrc_2=0;
    u16 RecvdCrc=0;
    /* Perform a Checksum on the frame*/
    TempVar_3 = CalculateCheksum(BLMGR_DataRxBuffer,FRAME_CHECKSUM_IDX -1u);
    if (TempVar_3 == BLMGR_DataRxBuffer[FRAME_CHECKSUM_IDX])
    {
        /*Perform Start and end of frame validation*/
        if((BLMGR_DataRxBuffer[FRAME_HEADER_IDX] == 0xaau) &&\
                (BLMGR_DataRxBuffer[FRAME_HEADER_IDX + 1u] == 0xaau) &&\
                (BLMGR_DataRxBuffer[FRAME_TAIL_IDX] == 0x55u))\
                        {
            /*Validate Frame Type*/
            if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == FRAME_TYPE_DATA_FRAME)
            {
                /* Check CRC*/
                /*Calculate Crc from received data*/
#if(defined (COMM_CINFIG) == defined (MSTER_COMM))
                TempBuffer_2[0x00] = BLMGR_DataRxBuffer[DIRECTION_IDX];
                TempBuffer_2[0x01] = BLMGR_DataRxBuffer[SPEED_DEGREE_IDX];
                SECR_GnerateCrc(TempBuffer_2, 2u, &GenCrc_2,BLMGR_CrcKey);
#elif(COMM_CINFIG == SLAVE_COMM)
                TempBuffer_2[0x00] = BLMGR_DataRxBuffer[BATT_LEVEL_IDX];
                SECR_GnerateCrc(TempBuffer_2, 1u, &GenCrc_2,BLMGR_CrcKey);
#else
                /*Wrong Config, State in Idle*/
                /*To do: Managing dev Errors*/
#endif
                /*Read Received CRC*/
                RecvdCrc = 0x00u;
                RecvdCrc = BLMGR_DataRxBuffer[FRAME_CRC_IDX];
                RecvdCrc |= (u16)(((u16)BLMGR_DataRxBuffer[FRAME_CRC_IDX + 1u]) << (u16)8);

                /*Compare the Two Crcs*/
                if(GenCrc_2 == RecvdCrc)
                {
                    /*Validate Frame Receiver*/
                    if(BLMGR_DataRxBuffer[FRAME_RECVER_IDX] == DEVICE_ROLE)
                    {
                        /*Update received paramters*/
                        /*Update Frame sender*/
                        BLMGR_RxFrameSender = BLMGR_DataRxBuffer[FRAME_SENDER_IDX];
#if(defined(COMM_CINFIG) == defined (MSTER_COMM))
                        /*Update Direction*/
                        BLMGR_RxDirection = BLMGR_DataRxBuffer[DIRECTION_IDX];
                        /*Update Speed degree*/
                        BLMGR_RxSpeedDegree = BLMGR_DataRxBuffer[SPEED_DEGREE_IDX];
#elif(COMM_CINFIG == SLAVE_COMM)
                        BLMGR_RxBattLevel = BLMGR_DataRxBuffer[BATT_LEVEL_IDX];
#else
                        /*Wrong Config, State in Idle*/
                        /*To do: Managing dev Errors*/
#endif
                        /*Update error state*/
                        BLMGR_ErrorState = ERRH_NO_ERROR;
                        IsFrameValid_3 = 1u;

                    }
                    else
                    {
                        /*Invalid Frame receiver*/
                        BLMGR_ErrorState = ERRH_INVALID_RECEIVER_ERROR;
                        IsFrameValid_3 = 0u;
                    }

                }

            }
            else
            {
                /*Invalid Frame Type*/
                BLMGR_ErrorState = ERRH_WRONG_FRAME_ERROR;
                IsFrameValid_3 = 0u;
            }
                        }
        else
        {
            /*Invalid Frame Detected*/
            BLMGR_ErrorState = ERRH_INVALID_FRAME_ERROR;
            IsFrameValid_3 = 0u;
        }
    }
    else
    {
        /*Checksum error*/
        BLMGR_ErrorState = ERRH_CHECKSUM_ERROR;
        IsFrameValid_3 = 0u;
    }
    return IsFrameValid_3;
}
/*********************************************************************************/
static void ErrorHandlingStateMachine(void)
{
    switch(BLMGR_ErrorState)
    {
    case ERRH_TIMEOUT_ERROR:
    {
        /*Check the Expected frame to be received*/
        switch(BLMGR_ExpectedReceivedFrame)
        {
        case FRAME_TYPE_ID_FRAME:
        {
            if(BLMGR_HandshakeFailRepCount <= MAX_HANDSHAKE_FAIL_REP)
            {
                BLMGR_HandshakeFailRepCount ++;
                BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_ID_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_RESEND_LAST_FRMAE);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_HandshakeFailRepCount = 0u;
                /*Handshaking failed*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
        }
        break;
        case FRAME_TYPE_VAL_FRAME:
        {
            if(BLMGR_HandshakeFailRepCount <= MAX_HANDSHAKE_FAIL_REP)
            {
                BLMGR_HandshakeFailRepCount ++;
                BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_VAL_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_RESEND_LAST_FRMAE);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_HandshakeFailRepCount = 0u;
                /*Handshaking failed*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
        }
        break;
        case FRAME_TYPE_DATA_FRAME:
        {


            if(BLMGR_CommFailReptCount <= MAX_COMM_FAIL_REP)
            {
                /*InserBreakPoint();*/
                BLMGR_CommFailReptCount ++;
                BLMGR_CommState = COMM_STATE_SEND_DATA_FRAME;
                /*Send Error frame*/
                /* UpdateErrorFrame(ERROR_TYPE_RESEND_LAST_FRMAE);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);*/
            }
            else
            {
                /*InserBreakPoint();*/
                BuzzerSound();
                BLMGR_CommFailReptCount = 0u;
                /*Handshaking failed*/
                BLMGR_CommState = COMM_STATE_FAILED;
            }
        }
        break;
        default:
            break;
        }

    }
    break;
    case ERRH_HANDSHAKE_ERROR:
    {
        if(BLMGR_HandshakeFailRepCount <= MAX_HANDSHAKE_FAIL_REP)
        {
            /*Start new handshaking*/
            BLMGR_HandshakeFailRepCount ++;
            BLMGR_HandShakeState = HANDSHAKE_STATE_IDLE;
            /*Send Error frame*/
            UpdateErrorFrame(ERROR_TYPE_START_NEW_HANDSHAKE);
            BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
            BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
        }
        else
        {
            BLMGR_HandshakeFailRepCount = 0u;
            /*Handshaking failed*/
            BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
        }
    }
    break;
    case ERRH_CHECKSUM_ERROR:
    {
        /*Check the Expected frame to be received*/
        switch(BLMGR_ExpectedReceivedFrame)
        {
        case FRAME_TYPE_ID_FRAME:
        {
            if(BLMGR_HandshakeFailRepCount <= MAX_HANDSHAKE_FAIL_REP)
            {
                BLMGR_HandshakeFailRepCount ++;
                BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_ID_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_RESEND_LAST_FRMAE);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_HandshakeFailRepCount = 0u;
                /*Handshaking failed*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
        }
        break;
        case FRAME_TYPE_VAL_FRAME:
        {
            if(BLMGR_HandshakeFailRepCount <= MAX_HANDSHAKE_FAIL_REP)
            {
                BLMGR_HandshakeFailRepCount ++;
                BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_VAL_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_RESEND_LAST_FRMAE);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_HandshakeFailRepCount = 0u;
                /*Handshaking failed*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
        }
        break;
        case FRAME_TYPE_DATA_FRAME:
        {
            if(BLMGR_CommFailReptCount <= MAX_COMM_FAIL_REP)
            {
                BLMGR_CommFailReptCount ++;
                BLMGR_CommState = COMM_STATE_RECV_DATA_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_RESEND_LAST_FRMAE);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_CommFailReptCount = 0u;
                /*Handshaking failed*/
                BLMGR_CommState = COMM_STATE_FAILED;
            }
        }
        break;
        default:
            break;
        }
    }
    break;
    case ERRH_INVALID_FRAME_ERROR:
    {
        switch(BLMGR_ExpectedReceivedFrame)
        {
        case FRAME_TYPE_ID_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
        }
        break;
        case FRAME_TYPE_VAL_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
        }
        break;
        case FRAME_TYPE_DATA_FRAME:
        {
            BLMGR_CommState = COMM_STATE_FAILED;
        }
        break;
        default:
            break;
        }
    }
    break;
    case ERRH_CRC_ERROR:
    {
        switch(BLMGR_ExpectedReceivedFrame)
        {
        case FRAME_TYPE_ID_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
        }
        break;
        case FRAME_TYPE_VAL_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
        }
        break;
        case FRAME_TYPE_DATA_FRAME:
        {
            BLMGR_CommState = COMM_STATE_FAILED;
        }
        break;
        default:
            break;
        }

    }
    break;
    case ERRH_INVALID_RECEIVER_ERROR:
    {
        /*Check the Expected frame to be received*/
        switch(BLMGR_ExpectedReceivedFrame)
        {
        case FRAME_TYPE_ID_FRAME:
        {
            if(BLMGR_HandshakeFailRepCount <= MAX_HANDSHAKE_FAIL_REP)
            {
                BLMGR_HandshakeFailRepCount ++;
                BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_ID_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_UPDATE_UR_TRANSMITTER);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,ID_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_HandshakeFailRepCount = 0u;
                /*Handshaking failed*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
        }
        break;
        case FRAME_TYPE_VAL_FRAME:
        {
            if(BLMGR_HandshakeFailRepCount <= MAX_HANDSHAKE_FAIL_REP)
            {
                BLMGR_HandshakeFailRepCount ++;
                BLMGR_HandShakeState = HANDSHAKE_STATE_RECV_VAL_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_UPDATE_UR_TRANSMITTER);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,VAL_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_HandshakeFailRepCount = 0u;
                /*Handshaking failed*/
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
        }
        break;
        case FRAME_TYPE_DATA_FRAME:
        {
            if(BLMGR_CommFailReptCount <= MAX_COMM_FAIL_REP)
            {
                BLMGR_CommFailReptCount ++;
                BLMGR_CommState = COMM_STATE_RECV_DATA_FRAME;
                /*Send Error frame*/
                UpdateErrorFrame(ERROR_TYPE_UPDATE_UR_TRANSMITTER);
                BLTD_StartReceivingData(BLMGR_DataRxBuffer,DATA_FRAME_LENGTH,&RxBufferDnCallBackNotif);
                BLTD_SendMessage(BLMGR_DataTxBuffer,ERROR_FRAME_LENGTH);
            }
            else
            {
                BLMGR_CommFailReptCount = 0u;
                /*Handshaking failed*/
                BLMGR_CommState = COMM_STATE_FAILED;
            }
        }
        break;
        default:
            break;
        }
    }
    break;
    case ERRH_WRONG_FRAME_ERROR:
    {
        if(BLMGR_DataRxBuffer[FRAME_TYPE_IDX] == FRAME_TYPE_ERROR_FRAME)
        {
            /*Handle Error frame*/
            CheckErrorFrame();
        }
        else
        {
            switch(BLMGR_ExpectedReceivedFrame)
            {
            case FRAME_TYPE_ID_FRAME:
            {
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
            break;
            case FRAME_TYPE_VAL_FRAME:
            {
                BLMGR_HandShakeState = HANDSHAKE_STATE_FAILED;
            }
            break;
            case FRAME_TYPE_DATA_FRAME:
            {
                BLMGR_CommState = COMM_STATE_FAILED;
            }
            break;
            default:
                break;
            }
        }
    }
    break;
    default:
        break;
    }
}
/*********************************************************************************/
static void UpdateErrorFrame(u8 ErrorType)
{
    /*Set Tx Frame to default values*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],TX_FRAME_DEFUALT,MAX_DATA_BUFFER_LENGTH);
    /*Set Header of Frame*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_HEADER_IDX],0xaau,2u);
    /*Set Device Sender*/
    BLMGR_DataTxBuffer[FRAME_SENDER_IDX] = DEVICE_ROLE;
    /*Set Device Receiver*/
    BLMGR_DataTxBuffer[FRAME_RECVER_IDX] = BLMGR_TxFrameReceiver;
    /*Set frame type*/
    BLMGR_DataTxBuffer[FRAME_TYPE_IDX] = FRAME_TYPE_ERROR_FRAME;
    /*Set paramter length*/
    BLMGR_DataTxBuffer[PARAM_LENGTH_IDX] = 1u;
    /*Set Error type*/
    BLMGR_DataTxBuffer[ERROR_TYPE_IDX] = ErrorType;
    /*Update Crc*/
    BLMGR_DataTxBuffer[FRAME_CRC_IDX] = TX_CRC_DEFAULT;
    BLMGR_DataTxBuffer[FRAME_CRC_IDX + 1u] = TX_CRC_DEFAULT;
    /*update Frame CheckSum*/
    BLMGR_DataTxBuffer[FRAME_CHECKSUM_IDX] = CalculateCheksum(BLMGR_DataTxBuffer,FRAME_CHECKSUM_IDX -1u);
    /*update frame tail*/
    MemSet(&BLMGR_DataTxBuffer[FRAME_TAIL_IDX],0x55u,1u);
}
/*********************************************************************************/
static void CheckErrorFrame(void)
{
    u8 ErrorType_1=0;
    ErrorType_1 = BLMGR_DataRxBuffer[ERROR_TYPE_IDX];
    switch(ErrorType_1)
    {
    case ERROR_TYPE_RESEND_LAST_FRMAE:
    {
        switch(BLMGR_ExpectedReceivedFrame)
        {
        case FRAME_TYPE_ID_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_ID_FRAME;
        }
        break;
        case FRAME_TYPE_VAL_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_VAL_FRMAE;
        }
        break;
        case FRAME_TYPE_DATA_FRAME:
        {
            BLMGR_CommState = COMM_STATE_SEND_DATA_FRAME;
        }
        break;
        default:
            break;
        }
    }
    break;
    case ERROR_TYPE_START_NEW_HANDSHAKE:
    {
        BLMGR_HandShakeState = HANDSHAKE_STATE_IDLE;
    }
    break;
    case ERROR_TYPE_UPDATE_UR_TRANSMITTER:
    {
        switch(BLMGR_ExpectedReceivedFrame)
        {
        case FRAME_TYPE_ID_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_ID_FRAME;
        }
        break;
        case FRAME_TYPE_VAL_FRAME:
        {
            BLMGR_HandShakeState = HANDSHAKE_STATE_SEND_VAL_FRMAE;
        }
        break;
        case FRAME_TYPE_DATA_FRAME:
        {
            BLMGR_CommState = COMM_STATE_SEND_DATA_FRAME;
        }
        break;
        default:
            break;
        }
    }
    break;
    default:
        break;
    }
}
/*********************************************************************************/
static void MemCpy( u8* DesPtr, const u8* SrcPtr, u8 Length)
{
    u16 LoopIndex_1=0u;
    for(LoopIndex_1 = 0u; LoopIndex_1 < Length; LoopIndex_1 ++)
    {
        DesPtr += LoopIndex_1;
        SrcPtr += LoopIndex_1;
        *(DesPtr) = *(SrcPtr);
    }
}
/*********************************************************************************/
static void MemSet(u8* DesPtr, u8 ConstVal, u8 Length)
{
    u16 LoopIndex_3=0u;
    for(LoopIndex_3 = 0u; LoopIndex_3 < Length; LoopIndex_3 ++)
    {
        DesPtr += LoopIndex_3;
        *(DesPtr) = ConstVal;
    }
}
/*********************************************************************************/
#if (defined (COMM_CINFIG) == defined (SLAVE_COMM))
static u8 MemCmp(const u8* Src1Ptr,const u8* Src2Ptr,u16 Length)
{
    u8 IsEqual = 1u;
    u8 LoopIndex_4=0u;
    for (LoopIndex_4 = 0u; (LoopIndex_4 < Length) && (IsEqual == 1u) ; LoopIndex_4 ++)
    {
        Src1Ptr += LoopIndex_4;
        Src2Ptr += LoopIndex_4;
        if(*(Src1Ptr) != *(Src2Ptr))
        {
            IsEqual = 0u;
        }
    }
    return IsEqual;
}
#endif
/*********************************************************************************/
static u8 CalculateCheksum(const u8* BufferPtr, u16 BufferLength)
{
    bool x_X;
    u8 Checksum = 0x00u;
    u16 LoopIndex_5;
    for (LoopIndex_5 = 0u; LoopIndex_5 <= BufferLength; LoopIndex_5 ++)
    {
        BufferPtr +=LoopIndex_5;
        Checksum += *(BufferPtr);
    }
   x_X = Checksum % 255u;
    return x_X;
}
/*********************************************************************************/
static u8 CheckCrc(void)
{
    u16 RxCrc=0u;
    u16 GenCrc_1=0u;
    u8 TempBuffer_10[MAX_DATA_BUFFER_LENGTH];
    u8 IsFrameValid_4=0u;
    RxCrc = 0x00u;
    RxCrc = BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX];
    RxCrc = (RxCrc | ((u16)BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX +1u])) << (u16)8;
    TempBuffer_10[0x00] = TX_OS_TYPE;
    TempBuffer_10[0x01] = TX_DEV_TYPE;
    MemCpy(&TempBuffer_10[0x02],BLMGR_TxDevicName,BLMGR_TxDeviceNameLength);
    SECR_GnerateCrc(TempBuffer_10,(u16)BLMGR_TxDeviceNameLength + 2u, &GenCrc_1,BLMGR_CrcKey);
    if(GenCrc_1 == RxCrc)
    {
        BLMGR_ErrorState = ERRH_NO_ERROR;
        IsFrameValid_4 = 1u;
    }
    else
    {
        /*Crc Error Found*/
        BLMGR_ErrorState = ERRH_CRC_ERROR;
        IsFrameValid_4 = 0u;
    }
    return IsFrameValid_4;
}
/*********************************************************************************/
#if (defined (COMM_CINFIG) == defined (SLAVE_COMM))
static u8 GetCrcKey(void)
{
    u8 IsFrameValid_5;
    u16 RxCrc_1;
    u16 GenCrc_3;
    static u8 TempBuffer_7[MAX_DATA_BUFFER_LENGTH];
    u8 LoopTerminated;
    u32 LoopIndex_6=0;
    RxCrc_1 = 0x00u;
    RxCrc_1 = BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX];
    RxCrc_1 = (RxCrc_1 |((u16)BLMGR_DataRxBuffer[FRAME_VAL_CRC_IDX +1u])) << 8u;
    TempBuffer_7[0x00] = TX_OS_TYPE;
    TempBuffer_7[0x01] = TX_DEV_TYPE;
    MemCpy(&TempBuffer_7[0x02],BLMGR_TxDevicName,BLMGR_TxDeviceNameLength);
    LoopTerminated = 0u;
    for (LoopIndex_6 = 0u; (LoopIndex_6 < 0xffffu) && (LoopTerminated == 0u); LoopIndex_6 ++)
    {
        SECR_GnerateCrc(TempBuffer_7,(u16)BLMGR_TxDeviceNameLength + 2u, &GenCrc_3,(LoopIndex_6 | 0x10000u));
        if(GenCrc_3 == RxCrc_1)
        {
            BLMGR_CrcKey = LoopIndex_6;
            LoopTerminated = 1u;
        }
    }
    if(LoopTerminated == 1u)
    {
        /*Done and CRC Key Found*/
        BLMGR_ErrorState = ERRH_NO_ERROR;
        IsFrameValid_5 = 1u;
    }
    else
    {
        /*Crc Error Found*/
        BLMGR_ErrorState = ERRH_CRC_ERROR;
        IsFrameValid_5 = 0u;
    }
    return IsFrameValid_5;
}
#endif
/*********************************************************************************/
static void PowerBluetoothOn(void)
{
    DIO_WritePort((u8)BlueToothPwrConfig.Portname,(u8)BLOUETOOTH_ON,(u8)BlueToothPwrConfig.PortMask1);
}
/*********************************************************************************/
static void PowerBluetoothOff(void)
{
    DIO_WritePort((u8)BlueToothPwrConfig.Portname,(u8)!BLOUETOOTH_ON,(u8)BlueToothPwrConfig.PortMask1);
}
/*********************************************************************************/
static void DisconnectStateMachine(void)
{
}
/*********************************************************************************/
static void DisconnectInit(void)
{
    BLMGR_DisconectionTimeOut = 0u;
}
/*********************************************************************************/
static void BuzzerInit(void)
{
    DIO_InitPortDirection(BuzzerConfig.Portname,0xffu,BuzzerConfig.PortMask1);
    DIO_WritePort(BuzzerConfig.Portname,0x00u,BuzzerConfig.PortMask1);
}
/*********************************************************************************/
static void PowerBlueToothInit(void)
{
    DIO_InitPortDirection(BlueToothPwrConfig.Portname,0xffu,BlueToothPwrConfig.PortMask1);
    DIO_WritePort(BlueToothPwrConfig.Portname,0x00u,BlueToothPwrConfig.PortMask1);
}
/*********************************************************************************/
static void BlueToothKeyInit(void)
{
    DIO_InitPortDirection(BluetoothKeyConfig.Portname,0xffu,BluetoothKeyConfig.PortMask1);
    DIO_WritePort(BluetoothKeyConfig.Portname,0xffu,BluetoothKeyConfig.PortMask1);
}

static void InserBreakPoint(void)
{
    DIO_WritePort(BuzzerConfig.Portname,0xffu,BuzzerConfig.PortMask1);
    while(1)
    {
    }
}
