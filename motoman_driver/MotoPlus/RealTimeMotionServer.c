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
void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller,
                                                 int sd);
void Ros_RealTimeMotionServer_StopConnection(Controller* controller,
                                             int connectionIndex);

// WaitForSimpleMsg Task:
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller,
                                               int connectionIndex);

// IncMove task
void Ros_RealTimeMotionServer_IncMoveLoopStart(Controller* controller);

// Simple messages
int Ros_RealTimeMotionServer_SimpleMsg_State(SimpleMsg* stateMsg,
                                             CtrlGroup* ctrlGroup,
                                             int sequence);

//-----------------------
// Function implementation
//-----------------------

void Ros_RealTimeMotionServer_StartNewConnection(Controller* controller,
                                                 int sd) {
  int groupNo;
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

  // Start the IncMoveTask on demand
  // If not started, start the IncMoveTask (there should be only one instance of
  // this thread)
  if (controller->tidIncMoveThread == INVALID_TASK) {
    puts("Creating new task: IncMoveTask");

    controller->tidIncMoveThread =
        mpCreateTask(MP_PRI_IP_CLK_TAKE, MP_STACK_SIZE,
                     (FUNCPTR)Ros_RealTimeMotionServer_IncMoveLoopStart,
                     (int)controller, 0, 0, 0, 0, 0, 0, 0, 0, 0);
    if (controller->tidIncMoveThread == ERROR) {
      puts(
          "Failed to create task for incremental-motion.  Check robot "
          "parameters.");
      mpClose(sd);
      controller->tidIncMoveThread = INVALID_TASK;
      Ros_Controller_SetIOState(IO_FEEDBACK_FAILURE, TRUE);
      mpSetAlarm(8004, "MOTOROS FAILED TO CREATE TASK", 4);

      return;
    }
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

void Ros_RealTimeMotionServer_StopConnection(Controller* controller,
                                             int connectionIndex) {}

//-----------------------------------------------------------------------
// Task that waits to receive new SimpleMessage and then processes it
//-----------------------------------------------------------------------
void Ros_RealTimeMotionServer_WaitForSimpleMsg(Controller* controller,
                                               int connectionIndex) {
  SimpleMsg receiveMsg;
  SimpleMsg replyMsg;
  int byteSize = 0, byteSizeResponse = 0;
  int minSize = sizeof(SmPrefix) + sizeof(SmHeader);
  int expectedSize;
  int ret = 0;
  BOOL bDisconnect = FALSE;
  int partialMsgByteCount = 0;
  BOOL bSkipNetworkRecv = FALSE;

  while (!bDisconnect)  // keep accepting messages until connection closes
  {
    // TODO(Lars): Process incoming messages. E.g. to start real-time control
  }

  Ros_Sleep(50);  // Just in case other associated task need time to clean-up.

  // close this connection
  Ros_RealTimeMotionServer_StopConnection(controller, connectionIndex);
}

void Ros_RealTimeMotionServer_IncMoveLoopStart(
    Controller* controller)  //<-- IP_CLK priority task
{
  int groupNo = 0;
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
  moveData.ctrl_grp |= (0x01 << groupNo);
  moveData.grp_pos_info[groupNo].pos_tag.data[0] =
      Ros_CtrlGroup_GetAxisConfig(controller->ctrlGroups[groupNo]);
  // Control with pulse increments
  moveData.grp_pos_info[0].pos_tag.data[3] = MP_INC_PULSE_DTYPE;

  while (timeoutCounter < REALTIME_MOTION_TIMEOUT_COUNTER_MAX) {
    // Sync with the interpolation clock
    mpClkAnnounce(MP_INTERPOLATION_CLK);

    // Populate state message and send to server
    memset(&stateMsg, CLEAR, sizeof(SimpleMsg));
    msgSize = Ros_RealTimeMotionServer_SimpleMsg_State(
        &stateMsg, controller->ctrlGroups[groupNo], sequence);
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

      SmBodyMotoMotionCtrl* motionCtrl;
      motionCtrl = &commandMsg.body.motionCtrl;

      // The received sequence number should match the state sequence number
      // sent in the same cycle
      if (stateMsg.body.motionReply.sequence == motionCtrl->sequence) {
        // Integrate rad/s -> rad for this cycle
        int i;
        for (i = 0; i < MAX_PULSE_AXES; i++) {
          motionCtrl->data[i] *= interpolPeriodSec;
        }

        printf("Sequence: %d\n", motionCtrl->sequence);

        // Convert from rad to pulse
        Ros_CtrlGroup_ConvertToMotoPos(controller->ctrlGroups[groupNo],
                                       commandMsg.body.motionCtrl.data,
                                       moveData.grp_pos_info[groupNo].pos);

      } else {
        // Stop motion if sequence numbers do not match
        memset(&moveData.grp_pos_info[groupNo].pos, CLEAR,
               sizeof(LONG) * MP_GRP_AXES_NUM);
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

    } else {
      timeoutCounter++;
      printf(
          "RealTimeMotionServer: Read from socket timed out. Timeout counter "
          "at %u\n",
          timeoutCounter);
    }
  }

  puts("Deleting RealTimeMotion server inc task");

  mpClose(sd);
  mpDeleteSelf;
}

int Ros_RealTimeMotionServer_SimpleMsg_State(SimpleMsg* stateMsg,
                                             CtrlGroup* ctrlGroup,
                                             int sequence) {
  // Initialize memory
  memset(stateMsg, CLEAR, sizeof(SimpleMsg));

  // Set prefix: Length of message excluding the prefix
  stateMsg->prefix.length = sizeof(SmHeader) + sizeof(SmBodyMotoMotionReply);

  // Set the information of the reply
  stateMsg->header.msgType = ROS_MSG_MOTO_MOTION_REPLY;
  stateMsg->header.commType = ROS_COMM_TOPIC;
  stateMsg->header.replyType = ROS_REPLY_INVALID;

  stateMsg->body.motionReply.groupNo = ctrlGroup->groupNo;
  stateMsg->body.motionReply.sequence = sequence;

  long pulsePos[MAX_PULSE_AXES];
  int bRet = Ros_CtrlGroup_GetFBPulsePos(ctrlGroup, pulsePos);
  if (bRet != TRUE) {
    return 0;
  }

  Ros_CtrlGroup_ConvertToRosPos(ctrlGroup, pulsePos,
                                stateMsg->body.motionReply.data);

  return stateMsg->prefix.length + sizeof(SmPrefix);
}
