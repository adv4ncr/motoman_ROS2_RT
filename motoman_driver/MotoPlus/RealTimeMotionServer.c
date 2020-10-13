// RealTimeMotionServer.c
//
/*
* Software License Agreement (BSD License) 
*
* Copyright (c) 2020, Norwegian University of Science and Technology (NTNU)
* All rights reserved.
*
* Redistribution and use in binary form, with or without modification,
* is permitted provided that the following conditions are met:
*
*       * Redistributions in binary form must reproduce the above copyright
*       notice, this list of conditions and the following disclaimer in the
*       documentation and/or other materials provided with the distribution.
*       * Neither the name of NTNU, nor the names 
*       of its contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/ 

#include "MotoROS.h"

//-----------------------
// Function Declarations
//-----------------------
// Main Task: 
void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller, int sd);
void Ros_RealTimeMotionServer_StopConnection(Controller* controller, int connectionIndex);

// WaitForSimpleMsg Task:
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller, int connectionIndex);

// IncMove task
void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller);


//-----------------------
// Function implementation
//-----------------------

void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller, int sd)
{

	int groupNo;
	int connectionIndex;
	
	printf("Starting new connection to the Real-Time Motion Server\r\n");

	//look for next available connection slot
	for (connectionIndex = 0; connectionIndex < MAX_MOTION_CONNECTIONS; connectionIndex++)
	{
		if (controller->sdMotionConnections[connectionIndex] == INVALID_SOCKET)
		{
			controller->sdMotionConnections[connectionIndex] = sd;
			break;
		}
	}
	
	if (connectionIndex == MAX_MOTION_CONNECTIONS)
	{
		puts("Motion server already connected... not accepting last attempt.");
		mpClose(sd);
		return;
	}


  // Start the IncMoveTask on demand 
	// If not started, start the IncMoveTask (there should be only one instance of this thread)
	//if(controller->tidIncMoveThread == INVALID_TASK)
	//{
	//	puts("Creating new task: IncMoveTask");
	//	
	//	controller->tidIncMoveThread = mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE, 
	//												(FUNCPTR)Ros_RealTimeMotionServer_IncMoveLoopStart,
	//												(int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	//	if (controller->tidIncMoveThread == ERROR)
	//	{
	//		puts("Failed to create task for incremental-motion.  Check robot parameters.");
	//		mpClose(sd);
	//		controller->tidIncMoveThread = INVALID_TASK;
	//		Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
	//		mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);

	//		return;
	//	}
	//}

	if (controller->tidMotionConnections[connectionIndex] == INVALID_TASK)
	{
		printf("Creating new task: tidMotionConnections (connectionIndex = %d)\n", connectionIndex);
		
			
		//start new task for this specific connection
		controller->tidMotionConnections[connectionIndex] = mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE, 
																		(FUNCPTR)Ros_RealTimeMotionServer_WaitForSimpleMsg,
																		(int)controller, connectionIndex, 0, 0, 0, 0, 0, 0, 0, 0);
	
		if (controller->tidMotionConnections[connectionIndex] != ERROR)
		{
			Ros_Controller_SetIOState(IO_FEEDBACK_MOTIONSERVERCONNECTED, TRUE); //set feedback signal indicating success
		}
		else
		{
			puts("Could not create new task in the motion server.  Check robot parameters.");
			mpClose(sd);
			controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;
			controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 6);
			return;
		}
	}

}

void Ros_RealTimeMotionServer_StopConnection(Controller* controller, int connectionIndex)
{

}

//-----------------------------------------------------------------------
// Task that waits to receive new SimpleMessage and then processes it
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller, int connectionIndex)
{
	SimpleMsg receiveMsg;
	SimpleMsg replyMsg;
	int byteSize = 0, byteSizeResponse = 0;
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;
	int ret = 0;
	BOOL bDisconnect = FALSE;
	int partialMsgByteCount = 0;
	BOOL bSkipNetworkRecv = FALSE;

	while(!bDisconnect) //keep accepting messages until connection closes
	{
    // TODO(Lars): Process incoming messages. E.g. to start real-time control

	}
	
	Ros_Sleep(50);	// Just in case other associated task need time to clean-up.
	
	//close this connection
	Ros_RealTimeMotionServer_StopConnection(controller, connectionIndex);
}

void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller) //<-- IP_CLK priority task
{
  int sd;
  struct sockaddr_in serverSockAddr;
  int sizeofSockAddr = sizeof(serverSockAddr);
  int ret;
  sd = mpSocket(AF_INET, SOCK_DGRAM, 0);
  if (sd < 0)
  {
    return;
  }

  // Set structure
  memset(&serverSockAddr, CLEAR, sizeof(struct sockaddr_in));
  serverSockAddr.sin_family = AF_INET;
  // TODO(Lars): Get the external server address from the TCP socket?
  serverSockAddr.sin_addr.s_addr = mpInetAddr("192.168.255.3");
  serverSockAddr.sin_port = mpHtons(50244);

  char buffer[REALTIME_MOTION_BUFFER_SIZE_MAX];
  memset(buffer, CLEAR, sizeof(buffer));
  int bytesRecv;
  int bytesSend;
  struct fd_set readFds;

  // Timeout counter for late packages from external control client
  int timeoutCounter = 0;
  // Timeout read at 4 ms
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 4000;

  // State
  MP_CTRL_GRP_SEND_DATA ctrlGrpSendData;
  MP_FB_PULSE_POS_RSP_DATA fbPulsePosRspData;

  // Command
  MP_EXPOS_DATA moveData;
  memset(&moveData, CLEAR, sizeof(MP_EXPOS_DATA));
  // R1: Robot 1. TODO(Lars): Add control of additional groups
  moveData.ctrl_grp = 1;
  // Control all six axes: 0x3f = 00111111
  moveData.grp_pos_info[0].pos_tag.data[0] = 0x3f;
  // Control with pulse increments
  moveData.grp_pos_info[0].pos_tag.data[3] = MP_INC_PULSE_DTYPE;

  LONG status;

  printf("Starting control loop with cycle time: %u ms\n", controller->interpolPeriod);

  FOREVER
  {
    mpClkAnnounce(MP_INTERPOLATION_CLK);

    if (timeoutCounter < REALTIME_MOTION_TIMEOUT_COUNTER_MAX)
    {
      // Send state
      ctrlGrpSendData.sCtrlGrp = 0;
      memset(&fbPulsePosRspData, CLEAR, sizeof(fbPulsePosRspData));
      status = mpGetFBPulsePos(&ctrlGrpSendData, &fbPulsePosRspData);
      if (status != OK)
      {
        printf("Failed to get pulse feedback position: %u\n", status);
      }
      memset(buffer, 0, sizeof(buffer));
      memcpy(&buffer, &fbPulsePosRspData.lPos, sizeof(fbPulsePosRspData.lPos));

      bytesSend =
          mpSendTo(sd, buffer,
                   sizeof(fbPulsePosRspData.lPos), 0,
                   (struct sockaddr*)&serverSockAddr, sizeof(serverSockAddr));

      // Read command from socket with timeout
       FD_ZERO(&readFds);
       FD_SET(sd, &readFds);
       if (mpSelect(sd + 1, &readFds, NULL, NULL, &tv) < 0)
       {
         timeoutCounter++;
         printf(
             "RealTimeMotionServer: Read from socket timedout. Timeout counter "
             "at %u\n",
             timeoutCounter);
         continue;
       }

       // Read velocity command from socket
       memset(buffer, CLEAR, sizeof(buffer));
       bytesRecv =
           mpRecvFrom(sd, buffer, REALTIME_MOTION_BUFFER_SIZE_MAX, 0,
                      (struct sockaddr*)&serverSockAddr, &sizeofSockAddr);
       if (bytesRecv < 0)
       {
         printf("RealTimeMotionServer: bytesRecv < 0\n");
       }
     

    // if(Ros_Controller_IsMotionReady(controller) 
    //   && !controller->bStopMotion)
    // {
    //   
    //   ret = mpExRcsIncrementMove(&moveData);
       //if(ret != 0)
       //{
       //	if(ret == -3)
       //		printf("mpExRcsIncrementMove returned: %d (ctrl_grp = %d)\r\n", ret, moveData.ctrl_grp);
       //	else
       //		printf("mpExRcsIncrementMove returned: %d\r\n", ret);
       //}
    // }
    }
  }
}
