/** \file rly08.cc
 * \author Robotnik Automation S.L.L.
 * \version 1.1
 * \date    2011
 *
 * \brief class for the rly08 control
 *
 * Copyright (c) 2012, Robotnik Automation, SLL
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Robotnik Automation, SLL. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
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
 *
 * \author Robotnik Automation SLL
 */

#include <rly_08/rly08.h>

/*!	\fn rly08::rly08(double hz)
 * 	\brief Public constructor
 */
rly08::rly08() : SerialDevice(RLY08_DEFAULT_PORT, RLY08_DEFAULT_TRANSFERRATE, RLY08_DEFAULT_PARITY, RLY08_DEFAULT_DATA_SIZE, RLY08_THREAD_DESIRED_HZ)
{
	sComponentName.assign("rly08");
	bRunning = false;
	this->iStatus = 0;
	// mutex intitialization
	pthread_mutex_init(&mutexSerial, NULL);
};

/*!	\fn rly08::rly08(const char *channel, int baud_rate, const char *parity, int datasize, double hz)
 * 	\brief Public constructor
 */
rly08::rly08(const char *device, int baud_rate, const char *parity, int datasize) : SerialDevice(device, baud_rate, parity, datasize, RLY08_THREAD_DESIRED_HZ)
{
	sComponentName.assign("rly08");
	bRunning = false;
	this->iStatus = 0;
	this->num_of_reading_errors = 0;
	// mutex intitialization
	pthread_mutex_init(&mutexSerial, NULL);
};

/*!	\fn rly08::rly08(const char *device, double hz)
 * 	\brief Public constructor
 */
rly08::rly08(const char *device, double hz) : SerialDevice(device, RLY08_DEFAULT_TRANSFERRATE, RLY08_DEFAULT_PARITY, RLY08_DEFAULT_DATA_SIZE, hz)
{
	sComponentName.assign("rly08");
	bRunning = false;
	this->iStatus = 0;
	this->num_of_reading_errors = 0;
	// mutex intitialization
	pthread_mutex_init(&mutexSerial, NULL);
};

/*!	\fn rly08::~rly08()
 * 	\brief Public destructor
 */
rly08::~rly08()
{

	pthread_mutex_destroy(&mutexSerial);
}

/*! \fn ReturnValue rly08::Start()
 * Starts the control thread of the component and its subcomponents
 * \return OK
 * \return RUNNING if it's already running
 * \return NOT_INITIALIZED if the component is not initialized
 */
ReturnValue rly08::Start()
{
	if (SerialDevice::Start() == OK)
	{
		bRunning = true;
		return OK;
	}
	return ERROR;
}

/*!	\fn void rly08::InitState()
 * 	\brief Actions in the initial state
 *
 */
void rly08::InitState()
{
	ReadSwVersion();

	SwitchToState(READY_STATE);
}

/*!	\fn void rly08::ReadyState()
 * 	\brief Actions in Ready state

*/
void rly08::ReadyState()
{

	//    if (!bRunning) return;
	// Reads the status
	if (ReadRelayStatus() == -1)
	{
		this->num_of_reading_errors++;
		if (num_of_reading_errors > RLY08_MAX_ERRORS)
		{
			ROS_ERROR("rly08::ReadyState: %d cycles with errors", this->num_of_reading_errors);
			clock_gettime(threadData.pthreadPar.clock, &this->tRecovery);
			SwitchToState(FAILURE_STATE);
		}
	}
	else
		this->num_of_reading_errors = 0;
}

/*!	\fn void rly08::StandbyState()
 * 	\brief Actions in standby state
 */
void rly08::StandbyState()
{
}

/*!	\fn void rly08::FailureState()
 * 	\brief Actions in Failure State
 *         If time to recover, closes and tries to open the device again
 */
void rly08::FailureState()
{
	struct timespec tNow;
	long diff;

	clock_gettime(threadData.pthreadPar.clock, &tNow);

	diff = calcdiff(tNow, tRecovery);

	if (diff > RLY08_RECOVERY_TIME)
	{
		ROS_INFO("rly08::FailureState: Trying to recover from failure");
		this->Close();
		usleep(500000);
		if ((this->Open() == OK) && (this->Configure() == OK))
		{
			SwitchToState(READY_STATE);
			this->num_of_reading_errors = 0;
			// rlcLog->AddError((char*)"rly08::FailureState: Recovering from failure state");
		}
		tRecovery = tNow;
	}
}

/*!	\fn void rly08::RelayOn()
 * 	\brief Relay On
 *         Swich On iRelay (0=ALL ON)
 */
void rly08::RelayOn(int iRelay)
{
	int n = 0;
	char WriteBuffer[3] = "\0";

	WriteBuffer[0] = 100 + iRelay;
	pthread_mutex_lock(&mutexSerial);
	WritePort(WriteBuffer, &n, 1);
	ROS_INFO("rly08::Switch On Relay %d", iRelay);
	pthread_mutex_unlock(&mutexSerial);
}

/*!	\fn void rly08::RelayOff()
 * 	\brief Relay Off
 *         Swich Off iRelay (0=ALL Off)
 */
void rly08::RelayOff(int iRelay)
{
	int n = 0;
	char WriteBuffer[3] = "\0";
	WriteBuffer[0] = 110 + iRelay;

	pthread_mutex_lock(&mutexSerial);
	WritePort(WriteBuffer, &n, 1);
	ROS_INFO("rly08::Switch Off Relay %d", iRelay);
	pthread_mutex_unlock(&mutexSerial);
}

/*!	\fn void rly08::GetVersion()
 * 	\brief Returns the Version and ModuleID
 */
void rly08::GetSwVersion(int *iMod_ID, int *iVer)
{

	*iMod_ID = this->iModuleID;
	*iVer = this->iVersion;
}

/*!	\fn void rly08::ReadSwVersion()
 * 	\brief Performs the call to get Version and ModuleID
 */
void rly08::ReadSwVersion()
{
	//    char cAux[LOG_STRING_LENGTH] = "\0";
	int n = 0;
	char WriteBuffer[3] = "\0";
	char ReadBuffer[3] = "\0";

	WriteBuffer[0] = 90;
	pthread_mutex_lock(&mutexSerial);
	WritePort(WriteBuffer, &n, 1);
	usleep(500000);
	if ((ReadPort(ReadBuffer, &n, 64) == OK) && n > 0)
	{
		this->iModuleID = (int)ReadBuffer[0];
		this->iVersion = (int)ReadBuffer[1];
		// sprintf(cAux,"rly08::Get Version V:%d, ID:%d ",*iVer,*iMod_ID);
		// rlcLog->AddEvent(cAux);
	}
	else
	{
		ROS_ERROR("rly08::ReadSwVersion: Can't Get Version ");
		// rlcLog->AddError(cAux);
	}
	pthread_mutex_unlock(&mutexSerial);
	return;
}

/*!	\fn int rly08::GetRelaysStatus()
 *  \brief Gets Relays Status
 */
unsigned int rly08::GetRelayStatus()
{
	return this->iStatus;
}

/*!	\fn int rly08::ReadRelayStatus()
 *  \brief Reads Relays Status
 */
int rly08::ReadRelayStatus()
{
	int n = 0, ret = 0;
	char WriteBuffer[3] = "\0";
	char ReadBuffer[3] = "\0";

	WriteBuffer[0] = 91;

	pthread_mutex_lock(&mutexSerial);

	WritePort(WriteBuffer, &n, 1);

	if ((ReadPort(ReadBuffer, &n, 64) == OK) && n > 0)
	{
		this->iStatus = ReadBuffer[0];
		// ROS_INFO("rly08::Relays Status %d",iStatus);
	}
	else
	{
		ROS_ERROR("rly08::ReadRelayStatus: Can't Get Relays Status");
		ret = -1;
	}

	pthread_mutex_unlock(&mutexSerial);

	return ret;
}
