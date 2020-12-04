// RealTimeMotionServer.c
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
void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller,
                                                 int sd);

void Ros_RealTimeMotionServer_StopConnection(Controller* controller,
                                             int connectionIndex);

// WaitForSimpleMsg Task:
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller,
                                               int connectionIndex);
int Ros_RealTimeMotionServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);
int Ros_RealTimeMotionServer_MotionCtrlProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg);

// Start the real time control client which communicates with an external
// control server
BOOL Ros_RealTimeMotionServer_RealTimeMotionClientStart(Controller* controller);
// Stop the real time control client which communicates with an external control
// server
BOOL Ros_RealTimeMotionServer_RealTimeMotionClientStop(Controller* controller);

// Start the main control loop in real time mode
void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller);

// Simple messages
int Ros_RealTimeMotionServer_SimpleMsg_State(SimpleMsg* stateMsg,
                                             CtrlGroup* ctrlGroup,
                                             int sequence);

int Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(
    Controller* controller, int messageId, MotoRealTimeMotionMode mode,
    SimpleMsg* sendMsg);

//-----------------------
// Function implementation
//-----------------------

void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller,
                                                 int sd) {
  int connectionIndex;

  printf("Starting new connection to the Real-Time Motion Server\r\n");

  // look for next available connection slot
  for (connectionIndex = 0; connectionIndex < MAX_MOTION_CONNECTIONS;
       connectionIndex++) {
    if (controller->sdMotionConnections[connectionIndex] == INVALID_SOCKET) {
      controller->sdMotionConnections[connectionIndex] = sd;
      break;
    }
  }

  if (connectionIndex == MAX_MOTION_CONNECTIONS) {
    puts("Motion server already connected... not accepting last attempt.");
    mpClose(sd);
    return;
  }

  if (controller->tidMotionConnections[connectionIndex] == INVALID_TASK) {
    printf("Creating new task: tidMotionConnections (connectionIndex = %d)\n",
           connectionIndex);

    // start new task for this specific connection
    controller->tidMotionConnections[connectionIndex] =
        mpCreateTask(MP_PRI_TIME_NORMAL, MP_STACK_SIZE,
                     (FUNCPTR)Ros_RealTimeMotionServer_WaitForSimpleMsg,
                     (int)controller, connectionIndex, 0, 0, 0, 0, 0, 0, 0, 0);

    if (controller->tidMotionConnections[connectionIndex] != ERROR) {
      Ros_Controller_SetIOState(
          IO_FEEDBACK_MOTIONSERVERCONNECTED,
          TRUE);  // set feedback signal indicating success
    } else {
      puts(
          "Could not create new task in the motion server.  Check robot "
          "parameters.");
      mpClose(sd);
      controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;
      controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
      Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
      mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 6);
      return;
    }
  }
}

//-----------------------------------------------------------------------
// Close a connection along with all its associated task
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_StopConnection(Controller* controller, int connectionIndex)
{   
	int tid;
	
	printf("Closing Real Time Motion Server Connection\r\n");
	
	//close this connection
	mpClose(controller->sdMotionConnections[connectionIndex]);
	//mark connection as invalid
	controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;
	
  //set feedback signal
  Ros_Controller_SetIOState(IO_FEEDBACK_MOTIONSERVERCONNECTED, FALSE);
		
	// Stop message receiption task
	tid = controller->tidMotionConnections[connectionIndex];
	controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
	printf("Real Time Motion Server Connection Closed\r\n");
	
	mpDeleteTask(tid);
}


int Ros_RealTimeMotionServer_GetExpectedByteSizeForMessageType(SimpleMsg* receiveMsg, int recvByteSize)
{
	int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
	int expectedSize;

	switch (receiveMsg->header.msgType)
	{
	case ROS_MSG_MOTO_MOTION_CTRL:
		expectedSize = minSize + sizeof(SmBodyMotoMotionCtrl);
		break;
	case ROS_MSG_MOTO_MOTION_REPLY:
		expectedSize = minSize + sizeof(SmBodyMotoMotionReply);
		break;
	default: //invalid message type
		return -1;
	}
	return expectedSize;
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
		Ros_Sleep(0);	//give it some time to breathe, if needed
		
		if (!bSkipNetworkRecv)
		{
			if (partialMsgByteCount) //partial (incomplete) message already received
			{
				//Receive message from the PC
				memset((&receiveMsg) + partialMsgByteCount, 0x00, sizeof(SimpleMsg) - partialMsgByteCount);
				byteSize = mpRecv(controller->sdMotionConnections[connectionIndex], (char*)((&receiveMsg) + partialMsgByteCount), sizeof(SimpleMsg) - partialMsgByteCount, 0);
				if (byteSize <= 0)
					break; //end connection

				byteSize += partialMsgByteCount;
				partialMsgByteCount = 0;
			}
			else //get whole message
			{
				//Receive message from the PC
				memset(&receiveMsg, 0x00, sizeof(receiveMsg));
				byteSize = mpRecv(controller->sdMotionConnections[connectionIndex], (char*)(&receiveMsg), sizeof(SimpleMsg), 0);
				if (byteSize <= 0)
					break; //end connection
			}
		}
		else
		{
			byteSize = partialMsgByteCount;
			partialMsgByteCount = 0;
			bSkipNetworkRecv = FALSE;
		}

		// Determine the expected size of the message
		expectedSize = -1;
		if(byteSize >= minSize)
		{
			expectedSize = Ros_RealTimeMotionServer_GetExpectedByteSizeForMessageType(&receiveMsg, byteSize);

			if (expectedSize == -1)
			{
				printf("Unknown Message Received (%d)\r\n", receiveMsg.header.msgType);
				Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGTYPE, &replyMsg, 0);
			}			
			else if (byteSize >= expectedSize) // Check message size
			{
				// Process the simple message
				ret = Ros_RealTimeMotionServer_SimpleMsgProcess(controller, &receiveMsg, &replyMsg);
				if (ret == 1) //error during processing
				{
					bDisconnect = TRUE;
				}
				else if (byteSize > expectedSize) // Received extra data in single message
				{
					//Special case where ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX message could have different lengths
					if (receiveMsg.header.msgType == ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX &&
						byteSize == (int)(minSize + sizeof(SmBodyJointTrajPtFullEx)))
					{
						// All good
						partialMsgByteCount = 0;
					}
					else
					{
						// Preserve the remaining bytes and treat them as the start of a new message
						Db_Print("MessageReceived(%d bytes): expectedSize=%d, processing rest of bytes (%d, %d, %d)\r\n", byteSize, expectedSize, sizeof(receiveMsg), receiveMsg.body.jointTrajData.sequence, ((int*)((char*)&receiveMsg + expectedSize))[5]);
						partialMsgByteCount = byteSize - expectedSize;
						memmove(&receiveMsg, (char*)&receiveMsg + expectedSize, partialMsgByteCount);

						//Did I receive multiple full messages at once that all need to be processed before listening for new data?
						if (partialMsgByteCount >= minSize)
						{
							expectedSize = Ros_RealTimeMotionServer_GetExpectedByteSizeForMessageType(&receiveMsg, partialMsgByteCount);
							bSkipNetworkRecv = (partialMsgByteCount >= expectedSize); //does my modified receiveMsg buffer contain a full message to process?
						}
					}
				}
				else // All good
					partialMsgByteCount = 0;
			}
			else // Not enough data to process the command
			{
				Db_Print("MessageReceived(%d bytes): expectedSize=%d\r\n", byteSize, expectedSize);
				Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
			}
		}
		else // Didn't even receive a command ID
		{
			Db_Print("Unknown Data Received (%d bytes)\r\n", byteSize);
			Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
		}

		//Send reply message
		byteSizeResponse = mpSend(controller->sdMotionConnections[connectionIndex], (char*)(&replyMsg), replyMsg.prefix.length + sizeof(SmPrefix), 0);
		if (byteSizeResponse <= 0)
			break;	// Close the connection
	}
	
	Ros_Sleep(50);	// Just in case other associated task need time to clean-up.
	
	//close this connection
	Ros_RealTimeMotionServer_StopConnection(controller, connectionIndex);
}

//-----------------------------------------------------------------------
// Checks the type of message and processes it accordingly
// Return -1=Failure; 0=Success; 1=CloseConnection; 
//-----------------------------------------------------------------------
int Ros_RealTimeMotionServer_SimpleMsgProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	int ret = 0;
	int invalidSubcode = 0;
	
	switch(receiveMsg->header.msgType)
	{
	case ROS_MSG_MOTO_MOTION_CTRL:
		ret = Ros_RealTimeMotionServer_MotionCtrlProcess(controller, receiveMsg, replyMsg);
		break;

	//-----------------------
	default:
		printf("Invalid message type: %d\n", receiveMsg->header.msgType);
		invalidSubcode = ROS_RESULT_INVALID_MSGTYPE;
		break;
	}
	
	// Check Invalid Case
	if(invalidSubcode != 0)
	{
		Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_INVALID, invalidSubcode, replyMsg, 0);
		ret = -1;
	}
		
	return ret;
}

//-----------------------------------------------------------------------
// Processes message of type: ROS_MSG_MOTO_MOTION_CTRL
// Return -1=Failure; 0=Success; 1=CloseConnection; 
//-----------------------------------------------------------------------
int Ros_RealTimeMotionServer_MotionCtrlProcess(Controller* controller, SimpleMsg* receiveMsg, 
										SimpleMsg* replyMsg)
{
	SmBodyMotoMotionCtrl* motionCtrl;

	//printf("In MotionCtrlProcess\r\n");

	// Check the command code
	motionCtrl = &receiveMsg->body.motionCtrl;
	switch(motionCtrl->command)
	{
    case ROS_CMD_START_RT_MODE:
		{
			BOOL bRet = Ros_RealTimeMotionServer_RealTimeMotionClientStart(controller);
			// Reply msg
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else 
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
    case ROS_CMD_STOP_RT_MODE:
		{
			// Stop Motion
      BOOL bRet = Ros_RealTimeMotionServer_RealTimeMotionClientStop(controller);
			// Reply msg
			if(bRet)
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_SUCCESS, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			else 
				Ros_SimpleMsg_MotionReply(receiveMsg, ROS_RESULT_FAILURE, 0, replyMsg, receiveMsg->body.motionCtrl.groupNo);
			break;
		}
	}
	return 0;
}

//-----------------------------------------------------------------------
// Start the real time control client which communicates with an external
// control server
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_RealTimeMotionClientStart(
    Controller* controller) {
  if (controller->tidIncMoveThread == INVALID_TASK) {
    puts("Creating new task: RealTimeMotionServer_IncMoveLoop");

    controller->tidIncMoveThread =
        mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE,
                     (FUNCPTR)Ros_RealTimeMotionServer_IncMoveLoopStart,
                     (int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (controller->tidIncMoveThread == ERROR) {
      puts(
          "Failed to create task for real time motion.  Check robot "
          "parameters.");
      controller->tidIncMoveThread = INVALID_TASK;
      Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
      mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);
      return FALSE;
    }
    return TRUE;
  }
  puts("RealTimeMotionServer_IncMoveLoop task already started!");
  return FALSE;
}

BOOL Ros_RealTimeMotionServer_RealTimeMotionClientStop(Controller* controller) {
  BOOL bStopped = FALSE;
  // Signal that the motion should be stopped
  controller->bStopMotion = TRUE;
  // Check that the motion has in fact been stopped
  int checkCnt;
  for (checkCnt = 0; checkCnt < MOTION_STOP_TIMEOUT; checkCnt++) {
    if (controller->tidIncMoveThread == INVALID_TASK) {
      bStopped = TRUE;
      // All motion should be stopped at this point, so turn of the flag
      controller->bStopMotion = FALSE;
      break;
    }
    Ros_Sleep(1);
  }
  return bStopped;
}

void Ros_RealTimeMotionServer_IncMoveLoopStart(
    Controller* controller)  //<-- IP_CLK priority task
{
  int groupNo;

  int bRet;
  long pulsePos[MAX_PULSE_AXES];
  long pulsePosCmd[MAX_PULSE_AXES];

  // Ensure that the motion is not stopped on the controller
  controller->bStopMotion = FALSE;

  float interpolPeriodSec = (float)controller->interpolPeriod * 0.001;

  // State and command messages
  SimpleMsg stateMsg;
  SimpleMsg commandMsg;
  int msgSize = 0;

  // UDP client socket
  int sd;
  sd = mpSocket(AF_INET, SOCK_DGRAM, 0);
  if (sd < 0) {
    return;
  }

  // Set up UDP server structure
  struct sockaddr_in serverSockAddr;
  int sizeofSockAddr = sizeof(serverSockAddr);
  memset(&serverSockAddr, CLEAR, sizeof(struct sockaddr_in));
  serverSockAddr.sin_family = AF_INET;
  serverSockAddr.sin_addr.s_addr =
      mpInetAddr("192.168.255.3");  // IP should really not be hardcoded here
  serverSockAddr.sin_port = mpHtons(REALTIME_MOTION_UDP_PORT);

  // Permanently specifies the server
  mpConnect(sd, &serverSockAddr, sizeofSockAddr);

  int bytesRecv;
  int bytesSend;

  // Timeout counter for late packages from external control client
  int timeoutCounter = 0;

  // Message id (sequence) that is incremented in each iteration
  int sequence = 0;

  printf("Starting Real Time Motion Server task\r\n");
  printf("Starting control loop with cycle time: %u ms\n",
         controller->interpolPeriod);

  // Command
  MP_EXPOS_DATA moveData;
  memset(&moveData, CLEAR, sizeof(MP_EXPOS_DATA));
	int i;
	for(i=0; i<controller->numGroup; i++)
	{
		moveData.ctrl_grp |= (0x01 << i); 
		moveData.grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(controller->ctrlGroups[i]);
    // Control with pulse increments
    moveData.grp_pos_info[i].pos_tag.data[3] = MP_INC_ANGLE_DTYPE;
	}

  // Set mode to velocity control for now
  MotoRealTimeMotionMode mode = MOTO_REALTIME_MOTION_MODE_JOINT_VELOCITY;

  while (timeoutCounter < REALTIME_MOTION_TIMEOUT_COUNTER_MAX &&
         Ros_Controller_IsMotionReady(controller) && !controller->bStopMotion) {
    // Sync with the interpolation clock
    mpClkAnnounce(MP_INTERPOLATION_CLK);

    // Populate state message and send to server
    memset(&stateMsg, CLEAR, sizeof(SimpleMsg));
    msgSize = Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(
        controller, sequence, mode, &stateMsg);
    if (msgSize > 0) {
      bytesSend = mpSend(sd, (char*)&stateMsg, sizeof(SimpleMsg), 0);
    }

    // Define read timeout
    struct timeval tv;
    tv.tv_sec = 0;
    tv.tv_usec = REALTIME_MOTION_TIMEOUT_MS * 1000;

    // Recieve command message with timeout
    struct fd_set readFds;
    FD_ZERO(&readFds);
    FD_SET(sd, &readFds);
    if (mpSelect(sd + 1, &readFds, NULL, NULL, &tv) > 0) {
      memset(&commandMsg, CLEAR, sizeof(SimpleMsg));
      bytesRecv = mpRecv(sd, (char*)&commandMsg, sizeof(SimpleMsg), 0);
      if (bytesRecv < 0) {
        break;
      }

      SmBodyMotoRealTimeMotionJointCommandEx* command =
          &commandMsg.body.realTimeMotionJointCommandEx;

      // The received sequence number should match the state sequence number
      // sent in the same cycle
      if (stateMsg.body.realTimeMotionJointStateEx.messageId ==
          command->messageId) {
        printf("Message ID: %d\n", command->messageId);


        switch (mode) {
          case MOTO_REALTIME_MOTION_MODE_IDLE:
            // Fallthrough
          case MOTO_REALTIME_MOTION_MODE_JOINT_POSITION:
            //for (groupNo = 0; groupNo < controller->numGroup; groupNo++) {

            //  // Current position in pulses
            //  Ros_CtrlGroup_GetFBPulsePos(controller->ctrlGroups[groupNo], pulsePos);
            //  // Convert command from rad to pulse
            //  Ros_CtrlGroup_ConvertToMotoPos(
            //    controller->ctrlGroups[groupNo],
            //    command->jointCommandData[groupNo].command,
            //    pulsePosCmd);

            //  int axisNo;
            //  for (axisNo = 0; axisNo < MAX_PULSE_AXES; axisNo++) {
            //    moveData.grp_pos_info[groupNo].pos[axisNo] = pulsePos[axisNo] - pulsePosCmd[axisNo];
            //  }
            //}
            //break;
          case MOTO_REALTIME_MOTION_MODE_JOINT_VELOCITY:
          {
            for (groupNo = 0; groupNo < controller->numGroup; groupNo++) {
              
              int axisNo;
              for (axisNo = 0; axisNo < MAX_PULSE_AXES; axisNo++) {
                moveData.grp_pos_info[groupNo].pos[axisNo] = (LONG)(command->jointCommandData[groupNo].command[axisNo] / RAD_PER_DEGREE * controller->interpolPeriod * 10); 
              }
              printf("Command: %d\n", moveData.grp_pos_info[groupNo].pos[0]);

            }
            break;
          }
        }
      } else {
        // Stop motion if sequence numbers do not match
        for (groupNo = 0; groupNo < controller->numGroup; groupNo++) {
          memset(&moveData.grp_pos_info[groupNo].pos, CLEAR,
            sizeof(LONG) * MP_GRP_AXES_NUM);
        }
      }

      if (Ros_Controller_IsMotionReady(controller) &&
          !controller->bStopMotion) {
        int ret = mpExRcsIncrementMove(&moveData);
        if (ret != 0) {
          if (ret == -3) {
            printf("mpExRcsIncrementMove returned: %d (ctrl_grp = % d)\r\n",
                   ret, moveData.ctrl_grp);
          } else {
            printf("mpExRcsIncrementMove returned: %d\r\n", ret);
          }
        }
      }

      // Increment sequence number
      sequence++;
      // Reset timeoutCounter
      timeoutCounter = 0;
    } else {
      // Increment timeout counter
      timeoutCounter++;
      printf(
          "RealTimeMotionServer: Read from socket timed out. Timeout "
          "counter "
          "at %u\n",
          timeoutCounter);
    }
  }

  puts("Deleting RealTimeMotion server inc task");

  // Close UDP socket
  mpClose(sd);
  // Delete this task
  controller->tidIncMoveThread = INVALID_TASK;
  mpDeleteSelf;
}

int Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(
    Controller* controller, int messageId, MotoRealTimeMotionMode mode,
    SimpleMsg* sendMsg) {
  int bRet;
  long pulsePos[MAX_PULSE_AXES];
  long pulseSpeed[MAX_PULSE_AXES];
  int groupNo;

  // initialize memory
  memset(sendMsg, 0x00, sizeof(SimpleMsg));

  // set prefix: length of message excluding the prefix
  sendMsg->prefix.length =
      sizeof(SmHeader) + sizeof(SmBodyMotoRealTimeMotionJointStateEx);

  // set header information
  sendMsg->header.msgType = ROS_MSG_MOTO_REALTIME_MOTION_JOINT_STATE_EX;
  sendMsg->header.commType = ROS_COMM_TOPIC;
  sendMsg->header.replyType = ROS_REPLY_INVALID;

  // set number of valid groups
  sendMsg->body.realTimeMotionJointStateEx.numberOfValidGroups =
      controller->numGroup;

  // set unique message id
  sendMsg->body.realTimeMotionJointStateEx.messageId = messageId;

  // set control mode (idle, position, velocity)
  sendMsg->body.realTimeMotionJointStateEx.mode = mode;

  // Populate the state of all control groups
  for (groupNo = 0; groupNo < controller->numGroup; groupNo++) {
    sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].groupNo =
        groupNo;

    // feedback position
    bRet =
        Ros_CtrlGroup_GetFBPulsePos(controller->ctrlGroups[groupNo], pulsePos);
    if (bRet != TRUE) {
      return 0;
    }
    Ros_CtrlGroup_ConvertToRosPos(
        controller->ctrlGroups[groupNo], pulsePos,
        sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].pos);

    // servo speed
    bRet = Ros_CtrlGroup_GetFBServoSpeed(controller->ctrlGroups[groupNo],
                                         pulseSpeed);
    if (bRet == TRUE) {
      Ros_CtrlGroup_ConvertToRosPos(
          controller->ctrlGroups[groupNo], pulseSpeed,
          sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].vel);
    }
  }

  return sendMsg->prefix.length + sizeof(SmPrefix);
}
