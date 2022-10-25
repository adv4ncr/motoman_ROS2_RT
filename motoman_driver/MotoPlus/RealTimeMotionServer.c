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
void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller, int sd);
void Ros_RealTimeMotionServer_StopConnection(Controller* controller, int connectionIndex);

// WaitForSimpleMsg Task:
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller, int connectionIndex);
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
void Ros_RealTimeMotionServer_IncMoveLoopStart2(Controller* controller);

// Simple messages
//int Ros_RealTimeMotionServer_SimpleMsg_State(SimpleMsg* stateMsg, CtrlGroup* ctrlGroup, int sequence);

int Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(
		Controller* controller, int messageId, MotoRealTimeMotionMode mode,
		SimpleMsg* sendMsg);

// Handle Controller
BOOL Ros_RealTimeMotionServer_StartTrajMode(Controller* controller);
BOOL Ros_RealTimeMotionServer_StopTrajMode(Controller* controller);
BOOL Ros_RealTimeMotionServer_StopMotion(Controller* controller);

// Utility functions
STATUS Ros_RealTimeMotionServer_DisableEcoMode(Controller* controller);
void Ros_RealTimeMotionServer_PrintError(USHORT err_no, char* msgPrefix);
void Ros_RealTimeMotionServer_PrintPrevPos(Controller* controller);
void Ros_RealTimeMotionServer_PrintParams(Controller* controller);

//-----------------------
// Function implementation
//-----------------------

void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller, int sd)
{
	int connectionIndex;

	printf("[RT] Starting new connection to the Real-Time Motion Server\r\n");

	// Look for next available connection slot. There is only one.
	for (connectionIndex = 0; connectionIndex < MAX_MOTION_CONNECTIONS; connectionIndex++)
	{
		if (controller->sdMotionConnections[connectionIndex] == INVALID_SOCKET)
		{
			controller->sdMotionConnections[connectionIndex] = sd;
			break;
		}
	}

	// Abort if already connected
	if (connectionIndex == MAX_MOTION_CONNECTIONS) {
		puts("[RT] motion server already connected... not accepting last attempt.");
		mpClose(sd);
		return;
	}

	// If not started, start the WaitForSimpleMsg task
	if (controller->tidMotionConnections[connectionIndex] == INVALID_TASK) {
		printf("[RT] Creating new task: tidMotionConnections (connectionIndex = %d)\n", connectionIndex);

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
			puts("[RT] Could not create new task in the motion server.  Check robot parameters.");
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

	printf("[RT] Closing Real Time Motion Server Connection\r\n");

	//close this connection
	mpClose(controller->sdMotionConnections[connectionIndex]);
	//mark connection as invalid
	controller->sdMotionConnections[connectionIndex] = INVALID_SOCKET;

	//set feedback signal
	Ros_Controller_SetIOState(IO_FEEDBACK_MOTIONSERVERCONNECTED, FALSE);

	// Stop message receiption task
	tid = controller->tidMotionConnections[connectionIndex];
	controller->tidMotionConnections[connectionIndex] = INVALID_TASK;
	printf("[RT] Real Time Motion Server Connection Closed\r\n");

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
	printf("[RT] wait for simple message.\r\n"); // #TODO remove
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

		if (!bSkipNetworkRecv) //if I don't already have an extra complete packet buffered from the previous recv
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
				printf("[RT] Unknown Message Received (%d)\r\n", receiveMsg.header.msgType);
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
					////Special case where ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX message could have different lengths - this never happens in RT mode
					//if (receiveMsg.header.msgType == ROS_MSG_MOTO_JOINT_TRAJ_PT_FULL_EX &&
					//	byteSize == (int)(minSize + sizeof(SmBodyJointTrajPtFullEx)))
					//{
					//	// All good
					//	partialMsgByteCount = 0;
					//}
					//else
					//{

					// Preserve the remaining bytes and treat them as the start of a new message
					Db_Print("[RT] MessageReceived(%d bytes): expectedSize=%d, processing rest of bytes (%d, %d, %d)\r\n", byteSize, expectedSize, sizeof(receiveMsg), receiveMsg.body.jointTrajData.sequence, ((int*)((char*)&receiveMsg + expectedSize))[5]);
					partialMsgByteCount = byteSize - expectedSize;
					memmove(&receiveMsg, (char*)&receiveMsg + expectedSize, partialMsgByteCount);

					//Did I receive multiple full messages at once that all need to be processed before listening for new data?
					if (partialMsgByteCount >= minSize)
					{
						expectedSize = Ros_RealTimeMotionServer_GetExpectedByteSizeForMessageType(&receiveMsg, partialMsgByteCount);
						bSkipNetworkRecv = (partialMsgByteCount >= expectedSize); //does my modified receiveMsg buffer contain a full message to process?
					}
					//}
				}
				else // All good
					partialMsgByteCount = 0;
			}
			else // Not enough data to process the command
			{
				Db_Print("[RT]MessageReceived(%d bytes): expectedSize=%d\r\n", byteSize, expectedSize);
				Ros_SimpleMsg_MotionReply(&receiveMsg, ROS_RESULT_INVALID, ROS_RESULT_INVALID_MSGSIZE, &replyMsg, 0);
			}
		}
		else // Didn't even receive a command ID
		{
			Db_Print("[RT] Unknown Data Received (%d bytes)\r\n", byteSize);
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
		printf("[RT] Invalid message type: %d\n", receiveMsg->header.msgType);
		invalidSubcode = ROS_RESULT_INVALID_MSGTYPE;
		break;
	}

	// Check Invalid Case #TODO can be moved to default
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
int Ros_RealTimeMotionServer_MotionCtrlProcess(Controller* controller, SimpleMsg* receiveMsg, SimpleMsg* replyMsg)
{
	SmBodyMotoMotionCtrl* motionCtrl;

	//printf("In MotionCtrlProcess\r\n");

	// Check the command code
	motionCtrl = &receiveMsg->body.motionCtrl;
	Db_Print("[RT] MotionCtrl: groupNo: %d sequence: %d command: %d\n", motionCtrl->groupNo, motionCtrl->sequence, motionCtrl->command);

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
		default:
			printf("[RT] invalid motionCtrl command: %d", motionCtrl->command);
			break;
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
		puts("[RT] Creating new task: RealTimeMotionServer_IncMoveLoop");

		controller->tidIncMoveThread =
				mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE,
										 (FUNCPTR)Ros_RealTimeMotionServer_IncMoveLoopStart2,
										 (int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		if (controller->tidIncMoveThread == ERROR) {
			puts("[RT] Failed to create task for real time motion.  Check robot parameters.");
			controller->tidIncMoveThread = INVALID_TASK;
			Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
			mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);
			return FALSE;
		}
		return TRUE;
	}
	puts("[RT] RealTimeMotionServer_IncMoveLoop task already started!");
	return FALSE;
}

//-----------------------------------------------------------------------
// Stop the real time control client which communicates with an external
// control server
//-----------------------------------------------------------------------
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


//-----------------------------------------------------------------------
// Real time Incremental Move Loop -- to be removed
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller)  //<-- IP_CLK priority task
{
	int groupNo;							// Robot group number
	//int bRet;								// bRet? 
	//long pulsePos[MAX_PULSE_AXES];			// Pulse position array
	//long pulsePosCmd[MAX_PULSE_AXES];		// Pulse position command array

	// Ensure that the motion is not stopped on the controller
	controller->bStopMotion = FALSE;

	//float interpolPeriodSec = (float)controller->interpolPeriod * 0.001;	// interpolation period [sec]

	// State and command messages
	SimpleMsg stateMsg;						// struct SimpleMsg state variable
	SimpleMsg commandMsg;					// struct SimpleMsg command variable
	//int msgSize = 0;						// int message size

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
	serverSockAddr.sin_addr.s_addr = mpInetAddr("192.168.255.3");  // IP should really not be hardcoded here. True. #TODO
	serverSockAddr.sin_port = mpHtons(REALTIME_MOTION_UDP_PORT);

	// Permanently specifies the server
	mpConnect(sd, (struct sockaddr *)&serverSockAddr, sizeofSockAddr);

	int bytesRecv;							// UDP received bytes
	int bytesSend;							// UDP sent bytes

	// Timeout counter for late packages from external control client
	int timeoutCounter = 0;

	// Message id (sequence) that is incremented in each iteration
	int sequence = 0;

	printf("[RT] Starting control loop with cycle time: %u ms\n", controller->interpolPeriod);

	MP_EXPOS_DATA moveData;					// Incremental move data
	memset(&moveData, CLEAR, sizeof(MP_EXPOS_DATA));
	
	int i;
	for(i=0; i<controller->numGroup; i++)
	{
		moveData.ctrl_grp |= (0x01 << i);
		moveData.grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(controller->ctrlGroups[i]);
		// Control with pulse increments
		moveData.grp_pos_info[i].pos_tag.data[3] = MP_INC_ANGLE_DTYPE;
		// #TODO not set: tool info[2], user coordinate sys[4], 
	}

	// Set mode to velocity control for now #TODO hardcoded mode
	MotoRealTimeMotionMode mode = MOTO_REALTIME_MOTION_MODE_JOINT_POSITION;

	// RT loop variables
	struct timeval tv;						// Define read timeout
	tv.tv_sec = 0;
	tv.tv_usec = REALTIME_MOTION_TIMEOUT_MS * 1000;
	struct fd_set readFds;					// struct for handling socket file descriptor state
	SmBodyMotoRealTimeMotionJointCommandEx* command;	// MotionJointCommandEx pointer


	while (timeoutCounter < REALTIME_MOTION_TIMEOUT_COUNTER_MAX &&
				 Ros_Controller_IsMotionReady(controller) && !controller->bStopMotion) {

		// Sync with the interpolation clock
		mpClkAnnounce(MP_INTERPOLATION_CLK);

		// Populate state message and send to server
		memset(&stateMsg, CLEAR, sizeof(SimpleMsg));
		if (Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(controller, sequence, mode, &stateMsg) > 0)
		{
			bytesSend = mpSend(sd, (char*)&stateMsg, sizeof(SimpleMsg), 0);
		}

		// Clear
		FD_ZERO(&readFds);
		FD_SET(sd, &readFds);

		// Recieve command message with timeout 
		if (mpSelect(sd + 1, &readFds, NULL, NULL, &tv) > 0) 
		{
			memset(&commandMsg, CLEAR, sizeof(SimpleMsg));
			bytesRecv = mpRecv(sd, (char*)&commandMsg, sizeof(SimpleMsg), 0);
			if (bytesRecv < 0) {
				break;
			}

			// set command
			command = &commandMsg.body.realTimeMotionJointCommandEx;

			// The received sequence number should match the state sequence number sent in the same cycle
			if (stateMsg.body.realTimeMotionJointStateEx.messageId == command->messageId) 
			{
				printf("[RT] M_ID: %d\n", command->messageId); // #TODO remove prints from RT LOOP


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
							printf("[RT] Command: %d\n", moveData.grp_pos_info[groupNo].pos[0]);

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
						printf("[RT] mpExRcsIncrementMove returned: %d (ctrl_grp = % d)\r\n",
									 ret, moveData.ctrl_grp);
					} else {
						printf("[RT] mpExRcsIncrementMove returned: %d\r\n", ret);
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
			printf("[RT] Socket read timeout. Counter at %u\n", timeoutCounter);
		}
	}

	puts("Deleting RealTimeMotion server inc task");

	// Close UDP socket
	mpClose(sd);
	// Delete this task
	controller->tidIncMoveThread = INVALID_TASK;
	mpDeleteSelf;
}

//-----------------------------------------------------------------------
// Real time Incremental Move Loop 2 -- new one
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_IncMoveLoopStart2(Controller* controller)  //<-- IP_CLK priority task
{
	UINT8 groupNo;							// Robot group number
	UINT8 axisNo;
	long pulsePos[MAX_PULSE_AXES];			// Pulse position array
	//long pulsePosTmp[MAX_PULSE_AXES];	// REMOVE #TEST2
	long pulsePosCmd[MAX_PULSE_AXES];		// Pulse position command array
	long pulseInc[MAX_PULSE_AXES];			// Pulse increment array
	INT8 init_motion = 0;					// Initial position alignment check
	struct timeval tv;						// Define UDP read timeout
	struct fd_set readFds;					// struct for handling socket file descriptor state
	
	UINT8 skip_next_send = 0;

	// Ensure that the motion is not stopped on the controller
	controller->bStopMotion = FALSE;

	//float interpolPeriodSec = (float)controller->interpolPeriod * 0.001;	// interpolation period [sec]

	// State and command messages
	SimpleMsg stateMsg;						// struct SimpleMsg state variable
	SimpleMsg commandMsg;					// struct SimpleMsg command variable
	// state message size
	const unsigned int stateMsgSize = sizeof(SmPrefix) + sizeof(SmHeader) + sizeof(SmBodyMotoRealTimeMotionJointStateEx);

	// UDP server socket
	int udp_socket_fd;
	if ((udp_socket_fd = mpSocket(AF_INET, SOCK_DGRAM, 0)) < 0)
	{
		printf("[RT] UDP socket creation failed. ERRNO: %d\n", errno); // #TODO better exit?
		goto exitTask;
	}

	// Set up UDP server structure
	struct sockaddr_in serverAddr, clientAddr;
	int sizeofClientAddr = sizeof(clientAddr);
	memset(&serverAddr, CLEAR, sizeof(serverAddr));
	memset(&clientAddr, CLEAR, sizeof(clientAddr));

	serverAddr.sin_family = AF_INET;
	serverAddr.sin_addr.s_addr = INADDR_ANY;
	serverAddr.sin_port = mpHtons(REALTIME_MOTION_UDP_PORT);

	// Bind server port
	if (mpBind(udp_socket_fd, (struct sockaddr *)&serverAddr, sizeof(serverAddr)) < 0)
	{
		printf("[RT] UDP bind failed. ERRNO: %d\n", errno);
		mpClose(udp_socket_fd);
		goto exitTask;
	}

	int bytesRecv;							// UDP received bytes
	int bytesSend;							// UDP sent bytes


	// ------------------ GET CLIENT INFORMATION ------------------
	FD_ZERO(&readFds);
	FD_SET(udp_socket_fd, &readFds);
	tv.tv_sec = 10; tv.tv_usec = 0;
	if (mpSelect(udp_socket_fd + 1, &readFds, NULL, NULL, &tv) <= 0)
	{
		printf("[RT] UDP request error or timeout %ld s\n", tv.tv_sec);
		goto exitTask;
	}

	char buf[1];
	UINT32 testRunSec = 0; // #TEST1
	bytesRecv = mpRecvFrom(udp_socket_fd, buf, 1, 0, (struct sockaddr *)&clientAddr, &sizeofClientAddr);
	if(bytesRecv < 0)
	{
		printf("[RT] UDP request client failed. ERRNO: %d\n", errno);
		goto exitTask;
	}
	else // successful
	{
		char str[INET_ADDRSTRLEN];
		mpInetNtoaB(clientAddr.sin_addr, str);
		printf("[RT] setting client IP: %s\n", str);
	}
	testRunSec = (unsigned char)buf[0] * 250;



	// ------------------ START CONTROLLER ------------------
	// Start Trajectory mode by starting the INIT_ROS job on the controller
	BOOL bRet = Ros_RealTimeMotionServer_StartTrajMode(controller);
	//BOOL bRet = TRUE; // #TODO remove #TEST1
	if (bRet)
	{
		Db_Print("[RT] controller active.\n");
		buf[0] = 0; // #TODO send more meaningful message
		bytesSend = mpSendTo(udp_socket_fd, buf, sizeof(buf), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
	}
	else
	{
		Db_Print("[RT] failed to start controller. Exit.\n");
		buf[0] = 1; // #TODO send more meaningful message
		bytesSend = mpSendTo(udp_socket_fd, buf, sizeof(buf), 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
		goto exitTask;
		// #TODO report to PC
	}

	// reply to confirm controller state


	// ------------------ VARIABLES ------------------
	// better move to other section? page faults due to memory locking? 

	// Timeout counter for late packages from external control client
	int timeoutCounter = 0;
	UINT32 runCounter = 0;

	// Message id (sequence) that is incremented in each iteration #TODO set to e.g. uint_8t
	int sequence = 0;


	MP_EXPOS_DATA moveData;					// Incremental move data
	memset(&moveData, CLEAR, sizeof(MP_EXPOS_DATA));

	int i;
	for (i = 0; i < controller->numGroup; i++)
	{
		moveData.ctrl_grp |= (0x01 << i);
		moveData.grp_pos_info[i].pos_tag.data[0] = Ros_CtrlGroup_GetAxisConfig(controller->ctrlGroups[i]);
		// Control with pulse increments
		//moveData.grp_pos_info[i].pos_tag.data[3] = MP_INC_ANGLE_DTYPE;
		moveData.grp_pos_info[i].pos_tag.data[3] = MP_INC_PULSE_DTYPE;
		// #TODO not set: tool info[2], user coordinate sys[4], 
	}

	// Set mode to velocity control for now #TODO hardcoded mode
	MotoRealTimeMotionMode mode = MOTO_REALTIME_MOTION_MODE_JOINT_POSITION;

	// RT loop variables
	tv.tv_sec = 0;
	tv.tv_usec = REALTIME_MOTION_TIMEOUT_MS * 1000;
	SmBodyMotoRealTimeMotionJointCommandEx* command;	// MotionJointCommandEx pointer

	// plot parameters
	Ros_RealTimeMotionServer_PrintParams(controller);

	// print prev pulses
	Ros_RealTimeMotionServer_PrintPrevPos(controller);

	// print start message
	printf("[RT] Starting control loop 2 with cycle time: %u ms\n", controller->interpolPeriod);


	// ------------------ MOTION LOOP ------------------

	while (timeoutCounter < REALTIME_MOTION_TIMEOUT_COUNTER_MAX &&
		runCounter < testRunSec && // #TEST1
		//Ros_Controller_IsMotionReady(controller) && // #TEST1
		!controller->bStopMotion) 
	{

		// Sync with the interpolation clock
		if (mpClkAnnounce(MP_INTERPOLATION_CLK) < 0) Db_Print("[RT] TASK TIMEOUT\n");

		// Populate state message and send to server
		
		memset(&stateMsg, CLEAR, stateMsgSize);
		if (Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(controller, sequence, mode, &stateMsg) > 0)
		{
			bytesSend = mpSendTo(udp_socket_fd, (char*)&stateMsg, stateMsgSize, 0, (struct sockaddr *)&clientAddr, sizeof(clientAddr));
		}


		// Clear
		FD_ZERO(&readFds);
		FD_SET(udp_socket_fd, &readFds);

		// Recieve command message with timeout 
		if (mpSelect(udp_socket_fd + 1, &readFds, NULL, NULL, &tv) > 0)
		{
			memset(&commandMsg, CLEAR, sizeof(SimpleMsg));
			bytesRecv = mpRecv(udp_socket_fd, (char*)&commandMsg, sizeof(SimpleMsg), 0);
			if (bytesRecv < 0) 
			{
				printf("[RT] mpRecv ERRNO: %d\n", errno);
				break;
			}
			// set command
			command = &commandMsg.body.realTimeMotionJointCommandEx;

			// The received sequence number should match the state sequence number sent in the same cycle
			if (stateMsg.body.realTimeMotionJointStateEx.messageId == command->messageId)
			{
				
				// check if first command sent is equal to start position
				if (init_motion == 0)
				{
					for (groupNo = 0; groupNo < controller->numGroup; groupNo++) {
						// get current actual pos in ticks
						Ros_CtrlGroup_GetFBPulsePos(controller->ctrlGroups[groupNo], pulsePos);
						// convert command to ticks
						Ros_CtrlGroup_ConvertToMotoPos(controller->ctrlGroups[groupNo], command->jointCommandData[groupNo].command, pulsePosCmd);
						for (axisNo = 0; axisNo < controller->ctrlGroups[groupNo]->numAxes; axisNo++) {
							// EXIT if deviation > START_MAX_PULSE_DEVIATION
							if (abs(pulsePosCmd[axisNo] - pulsePos[axisNo]) > START_MAX_PULSE_DEVIATION) {
								printf("[RT] ERROR. Deviation axis: %d POS: %ld - CMD_POS: %ld\n", axisNo, pulsePos[axisNo], pulsePosCmd[axisNo]);
								goto exitTask;
							}
							
						}

						Db_Print("[RT] Deviation OK\n");
					}
					init_motion++;
				}

				// switch motion types - currently only position
				switch (mode) {
				case MOTO_REALTIME_MOTION_MODE_IDLE:
					break;
				case MOTO_REALTIME_MOTION_MODE_JOINT_POSITION:
					for (groupNo = 0; groupNo < controller->numGroup; groupNo++) 
					{
						// Current position in pulses -> trust the robot instead
						// Ros_CtrlGroup_GetFBPulsePos(controller->ctrlGroups[groupNo], pulsePos);
						
						// Convert command from rad to pulse
						Ros_CtrlGroup_ConvertToMotoPos(controller->ctrlGroups[groupNo], command->jointCommandData[groupNo].command, pulsePosCmd);

						for (axisNo = 0; axisNo < controller->ctrlGroups[groupNo]->numAxes; axisNo++)
						{
							pulseInc[axisNo] = pulsePosCmd[axisNo] - pulsePos[axisNo];


							// set increment
							moveData.grp_pos_info[groupNo].pos[axisNo] = pulseInc[axisNo];

							// set previous (pulsePos) position
							pulsePos[axisNo] = pulsePosCmd[axisNo]; //-> better set after actual increment ?
						}
					}
					// print the deviation
					printf("[RT] P: %ld %ld %ld %ld %ld %ld\n",
						pulseInc[0], pulseInc[1], pulseInc[2], pulseInc[3], pulseInc[4], pulseInc[5]);

					break;
				case MOTO_REALTIME_MOTION_MODE_JOINT_VELOCITY:
					//for (groupNo = 0; groupNo < controller->numGroup; groupNo++) 
					//{
					//	UINT8 axisNo;
					//	for (axisNo = 0; axisNo < controller->ctrlGroups[groupNo]->numAxes; axisNo++) 
					//	{
					//		moveData.grp_pos_info[groupNo].pos[axisNo] = (LONG)(command->jointCommandData[groupNo].command[axisNo] / RAD_PER_DEGREE * controller->interpolPeriod * 10);
					//	}
					//}
					break;
				}
			}
			else 
			{
				// Stop motion if sequence numbers do not match #TODO!
				for (groupNo = 0; groupNo < controller->numGroup; groupNo++) {
					memset(&moveData.grp_pos_info[groupNo].pos, CLEAR,
						sizeof(LONG) * MP_GRP_AXES_NUM);
				}
				Db_Print("[RT] STE: %d CMD: %d\n",
					stateMsg.body.realTimeMotionJointStateEx.messageId, command->messageId);
			}

			// Move robot #TEST2
			if (Ros_Controller_IsMotionReady(controller) && !controller->bStopMotion) {
				int ret = mpExRcsIncrementMove(&moveData);
				if (ret != 0) {
					if (ret == -3) {
						printf("[RT] mpExRcsIncrementMove returned: %d (ctrl_grp = % d)\r\n",
							ret, moveData.ctrl_grp);
					}
					else {
						printf("[RT] mpExRcsIncrementMove returned: %d\r\n", ret);
					}
				}
				// normal end
				else
				{
					for (groupNo = 0; groupNo < controller->numGroup; groupNo++)
					{
						// #TODO
						// save current commanded pos to pulsePos. Does this work??
						//memcpy(pulsePosTmp, pulsePos, MAX_PULSE_AXES * sizeof(long));
						//Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[groupNo], pulsePos);
					}
				}
			}

			// Increment sequence number
			sequence++;
			// Reset timeoutCounter
			timeoutCounter = 0;
		}
		else {
			// Increment timeout counter
			timeoutCounter++;
			printf("[RT] Socket read timeout. Counter at %u\n", timeoutCounter);
		}


		if (runCounter % 2500 == 0) printf("[RT] runCounter: %d\n", runCounter); // #TEST1
		// Increment run counter
		runCounter++;
	}


exitTask:

	// plot parameters on exit again
	Ros_RealTimeMotionServer_PrintParams(controller);

	// print prev pulses
	Ros_RealTimeMotionServer_PrintPrevPos(controller);

	// Stop servos
	Ros_RealTimeMotionServer_StopTrajMode(controller);

	puts("[RT] Deleting RealTimeMotion server inc 2 task");

	// Close UDP socket
	mpClose(udp_socket_fd);
	// Delete this task
	controller->tidIncMoveThread = INVALID_TASK;
	mpDeleteSelf;
}



int Ros_RealTimeMotionServer_MotoRealTimeMotionJointStateEx(
		Controller* controller, int messageId, MotoRealTimeMotionMode mode,
		SimpleMsg* sendMsg) 
{
	int bRet;
	long pulsePos[MAX_PULSE_AXES];
	long pulseSpeed[MAX_PULSE_AXES];
	int groupNo;

	// initialize memory
	memset(sendMsg, 0x00, sizeof(SimpleMsg));

	// set prefix: length of message excluding the prefix
	sendMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoRealTimeMotionJointStateEx);

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
	for (groupNo = 0; groupNo < controller->numGroup; groupNo++) 
	{
		sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].groupNo =
				groupNo;

		// feedback position
		bRet = Ros_CtrlGroup_GetFBPulsePos(controller->ctrlGroups[groupNo], pulsePos);
		if (bRet != TRUE) {
			return 0;
		}
		Ros_CtrlGroup_ConvertToRosPos(
				controller->ctrlGroups[groupNo], pulsePos,
				sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].pos);

		// servo speed
		bRet = Ros_CtrlGroup_GetFBServoSpeed(controller->ctrlGroups[groupNo], pulseSpeed);
		if (bRet == TRUE) {
			Ros_CtrlGroup_ConvertToRosPos(
				controller->ctrlGroups[groupNo], pulseSpeed,
				sendMsg->body.realTimeMotionJointStateEx.jointStateData[groupNo].vel);
		}
	}

	return sendMsg->prefix.length + sizeof(SmPrefix);
}


//-----------------------------------------------------------------------
// Attempts to start playback of a job to put the controller in RosMotion mode
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_StartTrajMode(Controller* controller)
{
	int ret;
	MP_STD_RSP_DATA rData;
	MP_START_JOB_SEND_DATA sStartData;
	int checkCount;
	int grpNo;
	STATUS status;

	
	Db_Print("[RT] In StartTrajMode\r\n");

	// Update status
	Ros_Controller_StatusUpdate(controller);

	// Reset PFL Activation Flag
	if (controller->bPFLduringRosMove)
		controller->bPFLduringRosMove = FALSE;

	// Reset Inc Move Error
	if (controller->bMpIncMoveError)
		controller->bMpIncMoveError = FALSE;

	// Check if already in the proper mode
	if (Ros_Controller_IsMotionReady(controller))
		return TRUE;

	// Check if currently in operation, we don't want to interrupt current operation
	if (Ros_Controller_IsOperating(controller))
		return FALSE;

	// Check for condition that need operator manual intervention	
	if (Ros_Controller_IsEStop(controller))
	{
		Db_Print("[RT] Controller ESTOP\r\n");
		return FALSE;
	}
	if (Ros_Controller_IsHold(controller))
	{
		Db_Print("[RT] Controller HOLD\r\n");
		return FALSE;
	}
	if (!Ros_Controller_IsRemote(controller))
	{
		Db_Print("[RT] Controller not in REMOTE mode\r\n");
		return FALSE;
	}

	// Check for condition that can be fixed remotely
	if (Ros_Controller_IsError(controller))
	{
		// Cancel error
		memset(&rData, 0x00, sizeof(rData));
		ret = mpCancelError(&rData);
		if (ret != 0) goto updateStatus;
	}

	// Check for condition that can be fixed remotely
	if (Ros_Controller_IsAlarm(controller))
	{
		// Reset alarm
		memset(&rData, 0x00, sizeof(rData));
		ret = mpResetAlarm(&rData);
		if (ret == 0)
		{
			// wait for the Alarm reset confirmation
			int checkCount;
			for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);

				if (Ros_Controller_IsAlarm(controller) == FALSE)
					continue;

				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
			if (Ros_Controller_IsAlarm(controller)) goto updateStatus;
		}
		else goto updateStatus;
	}

	// Servo On
	if (Ros_Controller_IsServoOn(controller) == FALSE)
	{
		MP_SERVO_POWER_SEND_DATA sServoData;
		memset(&sServoData, 0x00, sizeof(sServoData));

		status = Ros_RealTimeMotionServer_DisableEcoMode(controller);
		if (status == NG)
		{
			goto updateStatus;
		}

		sServoData.sServoPower = 1;  // ON
		memset(&rData, 0x00, sizeof(rData));
		ret = mpSetServoPower(&sServoData, &rData);
		if ((ret == 0) && (rData.err_no == 0))
		{
			// wait for the Servo On confirmation
			int checkCount;
			for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);

				if (Ros_Controller_IsServoOn(controller) == TRUE)
					break;

				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
			if (Ros_Controller_IsServoOn(controller) == FALSE)
				goto updateStatus;
		}
		else
		{
			Ros_RealTimeMotionServer_PrintError(rData.err_no, "Can't turn on servo because:");
			goto updateStatus;
		}
	}

	// make sure that there is no data in the queues
	if (Ros_MotionServer_HasDataInQueue(controller)) {

		Db_Print("StartTrajMode clearing leftover data in queue\r\n");

		Ros_MotionServer_ClearQ_All(controller);

		if (Ros_MotionServer_HasDataInQueue(controller))
			printf("WARNING: StartTrajMode has data in queue\r\n");
	}

	// have to initialize the prevPulsePos that will be used when interpolating the traj
	for (grpNo = 0; grpNo < MP_GRP_NUM; ++grpNo)
	{
		if (controller->ctrlGroups[grpNo] != NULL)
		{
			Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos);
		}
	}

	// Start Job
	memset(&rData, 0x00, sizeof(rData));
	memset(&sStartData, 0x00, sizeof(sStartData));
	sStartData.sTaskNo = 0;
	memcpy(sStartData.cJobName, MOTION_INIT_ROS_JOB, MAX_JOB_NAME_LEN);
	ret = mpStartJob(&sStartData, &rData);
	if ((ret != 0) || (rData.err_no != 0))
	{
		Ros_RealTimeMotionServer_PrintError(rData.err_no, "Can't start job because:");
		goto updateStatus;
	}

	// wait for the Motion Ready
	for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
	{
		// Update status
		Ros_Controller_StatusUpdate(controller);

		if (Ros_Controller_IsMotionReady(controller))
			return(TRUE);

		Ros_Sleep(MOTION_START_CHECK_PERIOD);
	}

updateStatus:
	// Update status
	Ros_Controller_StatusUpdate(controller);

	return (Ros_Controller_IsMotionReady(controller));
}


//-----------------------------------------------------------------------
// Set I/O signal matching the WAIT instruction to allow the controller 
// to resume job execution
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_StopTrajMode(Controller* controller)
{
	// Don't change mode if queue is not empty
	if (Ros_MotionServer_HasDataInQueue(controller))
	{
		//printf("Failed: Ros_MotionServer_HasDataInQueue is true\r\n");
		return FALSE;
	}

	// Stop motion
	if (!Ros_RealTimeMotionServer_StopMotion(controller))
	{
		//printf("Failed: Ros_RealTimeMotionServer_StopMotion is false\r\n");
		return FALSE;
	}

	// Set I/O signal
	Ros_Controller_SetIOState(IO_FEEDBACK_MP_INCMOVE_DONE, TRUE);

	return TRUE;
}


//-----------------------------------------------------------------------
// Stop motion by stopping message processing and clearing the queue
//-----------------------------------------------------------------------
BOOL Ros_RealTimeMotionServer_StopMotion(Controller* controller)
{
	// NOTE: for the time being, stop motion will stop all motion for all control group 
	BOOL bRet = TRUE;
	BOOL bStopped;
	int checkCnt;
	int groupNo;

	// Stop any motion from being processed further
	controller->bStopMotion = TRUE;

	// Check that background processing of message has been stopped
	for (checkCnt = 0; checkCnt < MOTION_STOP_TIMEOUT; checkCnt++)
	{
		bStopped = TRUE;
		for (groupNo = 0; groupNo < controller->numGroup; groupNo++)
			bStopped &= !controller->ctrlGroups[groupNo]->hasDataToProcess;
		if (bStopped)
			break;
		else
			Ros_Sleep(1);
	}

	// Clear queues
	if (Ros_MotionServer_HasDataInQueue(controller)) 
	{

		Db_Print("[RT] stop motion - clearing leftover data in queue\r\n");
		bRet = Ros_MotionServer_ClearQ_All(controller);
		if (Ros_MotionServer_HasDataInQueue(controller)) printf("[RT] WARNING: StartTrajMode has data in queue\r\n");
	}

	// All motion should be stopped at this point, so turn of the flag
	controller->bStopMotion = FALSE;

	if (checkCnt >= MOTION_STOP_TIMEOUT)
		printf("[RT] WARNING: Message processing not stopped before clearing queue\r\n");

	return(bStopped && bRet);
}


//-----------------------------------------------------------------------
// Utility function: Disable eco mode on controller
//-----------------------------------------------------------------------
STATUS Ros_RealTimeMotionServer_DisableEcoMode(Controller* controller)
{
	MP_SERVO_POWER_SEND_DATA sServoData;
	MP_STD_RSP_DATA rData;
	int ret;

	if (Ros_Controller_IsEcoMode(controller) == TRUE)
	{
		//toggle servos to disable energy-savings mode
		sServoData.sServoPower = 0;  // OFF
		memset(&sServoData, 0x00, sizeof(sServoData));
		memset(&rData, 0x00, sizeof(rData));
		ret = mpSetServoPower(&sServoData, &rData);
		if ((ret == 0) && (rData.err_no == 0))
		{
			// wait for the Servo/Eco OFF confirmation
			int checkCount;
			for (checkCount = 0; checkCount < MOTION_START_TIMEOUT; checkCount += MOTION_START_CHECK_PERIOD)
			{
				// Update status
				Ros_Controller_StatusUpdate(controller);

				if (Ros_Controller_IsEcoMode(controller) == FALSE)
					break;

				Ros_Sleep(MOTION_START_CHECK_PERIOD);
			}
		}
		else
		{
			Ros_RealTimeMotionServer_PrintError(rData.err_no, "Can't disable energy-savings mode because:");
			return NG;
		}
	}

	if (Ros_Controller_IsEcoMode(controller) == FALSE) return OK;
	else return NG;
}


//-----------------------------------------------------------------------
// Utility function: Print Error
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_PrintError(USHORT err_no, char* msgPrefix)
{
	char errMsg[ERROR_MSG_MAX_SIZE];
	memset(errMsg, 0x00, ERROR_MSG_MAX_SIZE);
	Ros_Controller_ErrNo_ToString(err_no, errMsg, ERROR_MSG_MAX_SIZE);
	printf("%s %s\r\n", msgPrefix, errMsg);
}


//-----------------------------------------------------------------------
// Utility function: Print Inc Pos
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_PrintPrevPos(Controller* controller)
{
	UINT8 grpNo;
	for (grpNo = 0; grpNo < controller->numGroup; ++grpNo)
	{
		if (controller->ctrlGroups[grpNo] != NULL)
		{
			Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos);
			CtrlGroup* ctrlGroup = controller->ctrlGroups[grpNo];
			printf("[RT] ctrlGroup->prevPulsePos: %ld, %ld, %ld, %ld, %ld, %ld, %ld, %ld\r\n",
				ctrlGroup->prevPulsePos[0], ctrlGroup->prevPulsePos[1], ctrlGroup->prevPulsePos[2],
				ctrlGroup->prevPulsePos[3], ctrlGroup->prevPulsePos[4], ctrlGroup->prevPulsePos[5],
				ctrlGroup->prevPulsePos[6], ctrlGroup->prevPulsePos[7]);
		}
	}
}


//-----------------------------------------------------------------------
// Utility function: Print Parameters
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_PrintParams(Controller* controller)
{
	UINT8 grpNo, i;
	for (grpNo = 0; grpNo < controller->numGroup; ++grpNo)
	{
		if (controller->ctrlGroups[grpNo] != NULL)
		{
			Ros_CtrlGroup_GetPulsePosCmd(controller->ctrlGroups[grpNo], controller->ctrlGroups[grpNo]->prevPulsePos);
			CtrlGroup* ctrlGroup = controller->ctrlGroups[grpNo];


			printf("[PRMTR] groupNo: %d groupId: %d numAxes: %d tool: %d\n", 
				ctrlGroup->groupNo, ctrlGroup->groupId, ctrlGroup->numAxes, ctrlGroup->tool);
			//printf("[PRMTR] groupId: %d\n", ctrlGroup->groupId);
			//printf("[PRMTR] numAxes: %d\n", ctrlGroup->numAxes);
			//for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] pulseToRad[%d]: %f\n", i, ctrlGroup->pulseToRad.PtoR[i]);
			//for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] pulseToMeter[%d]: %f\n", i, ctrlGroup->pulseToMeter.PtoM[i]);
			//for (i = 0; i < MAX_PULSE_AXES; i++)
			//{
			//	printf("[PRMTR] correctionData[%d]: bValid: %d ulSourceAxis: %d ulCorrectionAxis: %d fCorrectionRatio: %f\n", i, 
			//		ctrlGroup->correctionData.correction[i].bValid,
			//		ctrlGroup->correctionData.correction[i].ulSourceAxis,
			//		ctrlGroup->correctionData.correction[i].ulCorrectionAxis,
			//		ctrlGroup->correctionData.correction[i].fCorrectionRatio
			//	);
			//}
			for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] maxIncrement[%d]: %u\n", i, ctrlGroup->maxInc.maxIncrement[i]);
			//for (i = 0; i < MP_GRP_AXES_NUM; i++) printf("[PRMTR] maxSpeed[%d]: %f\n", i, ctrlGroup->maxSpeed[i]);
			//printf("[PRMTR] tool: %d\n", ctrlGroup->tool);
			//for (i = 0; i < MAX_PULSE_AXES; i++) printf("[PRMTR] axisType[%d]: %d\n", i, ctrlGroup->axisType.type[i]);
			
			printf("[PRMTR] percentage limit: %f\n", GP_getGovForIncMotion(grpNo));
			
		}
	}
}

