/*

  ******************************************************************************
  * @file     : CANTest.cpp
  * @Copyright: ViewTool
  * @Revision : ver 1.0
  * @Date     : 2014/12/29 9:27
  * @brief    : CANTest demo
  ******************************************************************************
  * @attention
  *
  * Copyright 2009-2014, ViewTool
  * http://www.viewtool.com/
  * All Rights Reserved
  *
  ******************************************************************************
  */

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "ControlCAN.h"
#include <stdio.h>

#define CAN_CALLBACK_READ_DATA 0
#define can_index 1
#define send_command 1
#define read_data 1


#ifdef CAN_CALLBACK_READ_DATA
void WINAPI GetDataCallback(uint32_t DevIndex,uint32_t CANIndex,uint32_t Len)
{
    printf("GetDataCallback\n");
    int ReadDataNum;
	int DataNum = VCI_GetReceiveNum(VCI_USBCAN2, 0, can_index);
	VCI_CAN_OBJ	*pCAN_ReceiveData = (VCI_CAN_OBJ *)malloc(DataNum*sizeof(VCI_CAN_OBJ));
    if((DataNum > 0)&&(pCAN_ReceiveData != NULL)){
        ReadDataNum = VCI_Receive(VCI_USBCAN2, 0, can_index, pCAN_ReceiveData, DataNum);
        for (int i = 0; i < ReadDataNum; i++)
        {
			printf("\n");
            printf("--CAN_ReceiveData.RemoteFlag = %d\n",pCAN_ReceiveData[i].RemoteFlag);
            printf("--CAN_ReceiveData.ExternFlag = %d\n",pCAN_ReceiveData[i].ExternFlag);
            printf("--CAN_ReceiveData.ID = 0x%X\n",pCAN_ReceiveData[i].ID);
            printf("--CAN_ReceiveData.DataLen = %d\n",pCAN_ReceiveData[i].DataLen);
            printf("--CAN_ReceiveData.Data:");
            for(int j=0;j<pCAN_ReceiveData[i].DataLen;j++){
                printf("%02X ",pCAN_ReceiveData[i].Data[j]);
            }
            printf("\n");
            printf("--CAN_ReceiveData.TimeStamp = %d\n\n",pCAN_ReceiveData[i].TimeStamp);
        }
    }
	free(pCAN_ReceiveData);
}
#endif

// Used to do stuff with the ReadData function
void ParseData(VCI_CAN_OBJ pCAN_ReceiveData){
  int FrameID = pCAN_ReceiveData.ID;

  // printf("FrameID: %x\n\n", FrameID);

  int Frame_last = FrameID & 0x0000000f; // Grabs the last bit

  // printf("Frame_last: %x", Frame_last);
  // Check the last frame to see what we are getting
  if (Frame_last == 0x01)
  {
    // Not sure what this ID is, printing anyway
    int id = (pCAN_ReceiveData.Data[0] << 8) | pCAN_ReceiveData.Data[1];
    printf("ID 1: %d\n\n", id);

  } else if (Frame_last == 0x02)
  {
    // Grab the voltage and current in/out
    float v_in  = ((pCAN_ReceiveData.Data[0] << 8) | (pCAN_ReceiveData.Data[1]))/100.0;
    float i_in  = ((pCAN_ReceiveData.Data[2] << 8) | (pCAN_ReceiveData.Data[3]))/100.0;
    float v_out = ((pCAN_ReceiveData.Data[4] << 8) | (pCAN_ReceiveData.Data[5]))/100.0;
    float i_out = ((pCAN_ReceiveData.Data[6] << 8) | (pCAN_ReceiveData.Data[7]))/100.0;

    printf("Input Voltage: %f V\n", v_in);
    printf("Input Current: %f A\n", i_in);
    printf("Output Voltage: %f V\n", v_out);
    printf("Output Current: %f A\n\n", i_out);

  } else if (Frame_last == 0x03)
  {
    float power_supply = ((pCAN_ReceiveData.Data[0] << 8) | (pCAN_ReceiveData.Data[1]))/1000.0;
    float coolant_temp = ((pCAN_ReceiveData.Data[2] << 8) | (pCAN_ReceiveData.Data[3]))/100.0;
    float internal_temp =((pCAN_ReceiveData.Data[4] << 8) | (pCAN_ReceiveData.Data[5]))/100.0;

    printf("Internal Power Supply %f V\n", power_supply);
    printf("Coolant Temp %f C\n", coolant_temp);
    printf("Internal Temp %f C\n\n", internal_temp);
  } else if (Frame_last == 0x0B)
  {
    printf("Error Message: ");
    for(int i = 0; i < pCAN_ReceiveData.DataLen; i++)
    {
      printf(" %x ", pCAN_ReceiveData.Data[i]);
    }
    printf("\n");
  }
}

// CAN READ DATA
void ReadData()
{
  int ReadDataNum;
  int DataNum = VCI_GetReceiveNum(VCI_USBCAN2, 0, can_index);
  VCI_CAN_OBJ	*pCAN_ReceiveData = (VCI_CAN_OBJ *)malloc(DataNum*sizeof(VCI_CAN_OBJ));

      // printf("DataNum: %i\n", DataNum);
    if((DataNum > 0)&&(pCAN_ReceiveData != NULL)){
    ReadDataNum = VCI_Receive(VCI_USBCAN2, 0, can_index, pCAN_ReceiveData, DataNum);
    for (int i = 0; i < ReadDataNum; i++){ParseData(pCAN_ReceiveData[i]);}
  }
  free(pCAN_ReceiveData);
}

// CAN SEND DATA
//Send data
void SendCommand(int Power, int Voltage)
{
  int Status;

  VCI_CAN_OBJ	CAN_SendData;

  CAN_SendData.ID = 0x1A01000B;

  CAN_SendData.ExternFlag = 0;
  CAN_SendData.RemoteFlag = 0;
  CAN_SendData.SendType = 2;
  CAN_SendData.SendType = 0;


  // Pack in the data bytes
  CAN_SendData.DataLen = 5;
  CAN_SendData.Data[0] = 0x00; // Power Operation Mode
  CAN_SendData.Data[1] = 0x00; //DCDC Status Field (Power Operation)
  CAN_SendData.Data[2] = 0x00;//Power; // Power level
  CAN_SendData.Data[3] = 0x80;//Voltage; // Output voltage
  CAN_SendData.Data[4] = 0x00; // Output Voltage Regulation
  CAN_SendData.Data[5] = 0x00;
  CAN_SendData.Data[6] = 0x00;
  CAN_SendData.Data[7] = 0x40;

  Status = VCI_Transmit(VCI_USBCAN2,0,can_index,&CAN_SendData,1);
  if(Status==STATUS_ERR){
      printf("Send CAN data failed!\n");
      VCI_ResetCAN(VCI_USBCAN2,0,can_index);
  } else{
      printf("Send CAN data success!\n");
    }
}


// CAN INIT Code in case INIT_EX too much
// VCI_INIT_CONFIG	CAN_Init;
//   //Config device
// CAN_Init.AccCode = 0x00000000;
// CAN_Init.AccMask = 0xFFFFFFFF;
// CAN_Init.Filter = 1;
// CAN_Init.Mode = 1;
// CAN_Init.Timing0 = 0x00;
// CAN_Init.Timing1 = 0x1C;
//   Status = VCI_InitCAN(VCI_USBCAN2,0,can_index,&CAN_Init);
//   if(Status==STATUS_ERR){
//       printf("Init device failed!\n");
//       return 0;
//   }else{
//       printf("Init device success!\n");
//   }


int main(void)
{
    int DevNum,Status;

    //Scan device
    DevNum = VCI_ScanDevice(1);
    if(DevNum > 0){
        printf("Have %d device connected!\n",DevNum);
    } else{
        printf("No device connected!\n");
        return 0;
    }

    //Open device
    Status = VCI_OpenDevice(VCI_USBCAN2,0,0);
    if(Status==STATUS_ERR){
        printf("Open device failed!\n");
        printf("Status Message: %i\n", Status);
        return 0;
    } else{
        printf("Open device success!\n");
    }

    // CAN INIT Ex
	  VCI_INIT_CONFIG_EX	CAN_InitEx;
    //Config device
    CAN_InitEx.CAN_ABOM = 0;
    CAN_InitEx.CAN_Mode = 0;
    //CAN baud rate = 36MHz/(CAN_BRP)/(CAN_SJW+CAN_BS1+CAN_BS2)
    //1Mbps
    CAN_InitEx.CAN_BRP = 12;//6;
    CAN_InitEx.CAN_BS1 = 3;//3;
    CAN_InitEx.CAN_BS2 = 2;//2;
    CAN_InitEx.CAN_SJW = 1;

    CAN_InitEx.CAN_NART = 0;
    CAN_InitEx.CAN_RFLM = 0;
    CAN_InitEx.CAN_TXFP = 1;
	  CAN_InitEx.CAN_RELAY = 0;
    Status = VCI_InitCANEx(VCI_USBCAN2,0,can_index,&CAN_InitEx);
    if(Status==STATUS_ERR){
        printf("Init device failed!\n");
        printf("Status Err: %i\n", Status);
        return 0;
    }else{
        printf("Init device success!\n");
    }
    //Set filter
	  VCI_FILTER_CONFIG CAN_FilterConfig;
    CAN_FilterConfig.FilterIndex = 0;
    CAN_FilterConfig.Enable = 1;		//Enable
    CAN_FilterConfig.ExtFrame = 1;
    CAN_FilterConfig.FilterMode = 0;
    CAN_FilterConfig.ID_IDE = 0;
    CAN_FilterConfig.ID_RTR = 0;
    CAN_FilterConfig.ID_Std_Ext = 0;
    CAN_FilterConfig.MASK_IDE = 0;
    CAN_FilterConfig.MASK_RTR = 0;
    CAN_FilterConfig.MASK_Std_Ext = 0;
    Status = VCI_SetFilter(VCI_USBCAN2,0,can_index,&CAN_FilterConfig);
    if(Status==STATUS_ERR){
        printf("Set filter failed!\n");
        return 0;
    }else{
        printf("Set filter success!\n");
    }

	   //Register callback function
    #if CAN_CALLBACK_READ_DATA
      VCI_RegisterReceiveCallback(0,GetDataCallback);
    #endif

    //Start CAN
    Status = VCI_StartCAN(VCI_USBCAN2,0,can_index);
    if(Status==STATUS_ERR){
        printf("Start CAN failed!\n");
        return 0;
    }else{
        printf("Start CAN success!\n");
    }

    // Send Commmand Block
    if(send_command){
      SendCommand(0x258,0x190);
    }

    // Read Data Block
    if(read_data){
      while(1){
        ReadData();
        //if(send_command){
        //    SendCommand(0x258,0x190);
        //}
      }
    }

    sleep(0.01);
    while(getchar()!='\n');

    #if CAN_CALLBACK_READ_DATA
	  VCI_LogoutReceiveCallback(0);
	  printf("VCI_LogoutReceiveCallback\n");
    #endif

	  usleep(10*1000);
	  //Stop receive can data
	  Status = VCI_ResetCAN(VCI_USBCAN2,0,can_index);
    printf("VCI_ResetCAN %d\n",Status);
	  VCI_CloseDevice(VCI_USBCAN2,0);
	  printf("VCI_CloseDevice\n");
    return 0;
}
