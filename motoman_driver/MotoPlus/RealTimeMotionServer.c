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
void Ros_RealTimeMotionServer_Start(Controller* controller, int sd);
void Ros_RealTimeMotionServer_Stop(Controller* controller, int connectionIndex);

// IncMove task
void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller);


//-----------------------
// Function implementation
//-----------------------

void Ros_RealTimeMotionServer_Start(Controller* controller, int sd)
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


	// If not started, start the IncMoveTask (there should be only one instance of this thread)
	if(controller->tidIncMoveThread == INVALID_TASK)
	{
		puts("Creating new task: IncMoveTask");
		
		controller->tidIncMoveThread = mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE, 
													(FUNCPTR)Ros_RealTimeMotionServer_IncMoveLoopStart,
													(int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		if (controller->tidIncMoveThread == ERROR)
		{
			puts("Failed to create task for incremental-motion.  Check robot parameters.");
			mpClose(sd);
			controller->tidIncMoveThread = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);

			return;
		}
	}

}

void Ros_RealTimeMotionServer_Stop(Controller* controller, int sd)
{

}

void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller) //<-- IP_CLK priority task
{
  struct sockaddr_in clientSockAddr;
  int sizeofSockAddr;

  MP_EXPOS_DATA moveData;

  MP_CTRL_GRP_SEND_DATA ctrlGrpSendData;
  MP_FB_PULSE_POS_RSP_DATA fbPulsePosRspData;

  int i;
  int ret;
  LONG status;

  memset(&clientSockAddr, CLEAR, sizeof(clientSockAddr));
  sizeofSockAddr = sizeof(clientSockAddr);

  memset(&moveData, CLEAR, sizeof(MP_EXPOS_DATA));

  // TODO(Lars): Add control of additional groups

  // R1: Robot 1
  moveData.ctrl_grp = 1;
  // Control all six axes: 0x3f = 00111111
  moveData.grp_pos_info[0].pos_tag.data[0] = 0x3f;
  // Control with pulse increments
  moveData.grp_pos_info[0].pos_tag.data[3] = MP_INC_PULSE_DTYPE;

  char buffer[REALTIME_MOTION_BUFFER_SIZE_MAX];
  int bytesRecv;
  int bytesSend;

  int timeoutCounter = 0;


  FOREVER
  {
    mpClkAnnounce(MP_INTERPOLATION_CLK);

    memset(buffer, CLEAR, sizeof(buffer));
    // Read velocity command from socket
    bytesRecv = mpRecvFrom(controller->sdMotionConnections[0], 
      buffer, REALTIME_MOTION_BUFFER_SIZE_MAX, 0, 
      (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
    if (bytesRecv < 0)
    {
      break;
    }


   // fd_set fds;
   // FD_ZERO(&fds);
   // FD_SET(controller->sdMotionConnections[0], &fds);

   // struct timeval tv;
   // tv.tv_sec = 0;
   // tv.tv_usec = 4000;

   // if (mpSelect(controller->sdMotionConnections[0] + 1, &fds, NULL, NULL, &tv) < 0)
   // {
   //   if (timeoutCounter < 5)
   //   {
   //     timeoutCounter++;
   //     continue;
   //   }
   //   break;
   // }

   // if (FD_ISSET(controller->sdMotionConnections[0], &fds))
   // {
   //   memset(buffer, CLEAR, sizeof(buffer));
   //   // Read velocity command from socket
   //   bytesRecv = mpRecvFrom(controller->sdMotionConnections[0], 
   //     buffer, REALTIME_MOTION_BUFFER_SIZE_MAX, 0, 
   //     (struct sockaddr *)&clientSockAddr, &sizeofSockAddr);
   //   if (bytesRecv < 0)
   //   {
   //     break;
   //   }
   // }

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

    ctrlGrpSendData.sCtrlGrp = 0;
    memset(&fbPulsePosRspData, CLEAR, sizeof(fbPulsePosRspData));
    status = mpGetFBPulsePos(&ctrlGrpSendData, &fbPulsePosRspData);
    if (0 != status) 
    {
      printf("Failed to get pulse feedback position: %u\n", status);
    }

    memset(buffer, 0, sizeof(buffer));
    memcpy(&buffer, &fbPulsePosRspData.lPos, sizeof(fbPulsePosRspData.lPos));

    bytesSend =
      mpSendTo(controller->sdMotionConnections[0], buffer, sizeof(fbPulsePosRspData.lPos), 0,
      (struct sockaddr *)&clientSockAddr, sizeof(clientSockAddr));
  }
}
