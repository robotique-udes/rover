/** \file rly08.h
 * \author Robotnik Automation S.L.L.
 * \version 2.0
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

#include <rly_08/SerialDevice.h>
#include <ros/ros.h>

#ifndef __rly08_H
	#define __rly08_H

//! Timeout for controlling the communication with the device (in uSecs)
#define RLY08_TIMEOUT_COMM			 	    30000
//! Time waiting after an error and before to try to recover (uSecs)
#define RLY08_RECOVERY_TIME                5000000
//! Desired frequency
#define RLY08_THREAD_DESIRED_HZ			    10.0

#define RLY08_DEFAULT_PORT 		 	        "/dev/ttyUSB0"
#define RLY08_DEFAULT_PARITY 		 	    "none" 	//"even" "odd""none"
#define RLY08_DEFAULT_TRANSFERRATE	 	    19200
#define RLY08_DEFAULT_DATA_SIZE			    8

#define RLY08_SERIAL_DELAY					10000		//! us between serial transmisions to the rly08
#define RLY08_MAX_ERRORS					20			// Max number of consecutive communications errors

#define RLY08_DEFAULT_OUTPUTS				8

//! Class to operate with the ms20 magnets sensor
class rly08: public SerialDevice {

private:
	//! Controls the execution of the thread
	bool bRunning;
	//! State of the controller
	unsigned int iStatus;
	//! Auxiliar variable for controling the timeout in the communication
	struct timespec tNext;
	//! Auxiliar variable for controling the timeout in the communication
	struct timespec	tNow;
	//! Time of the last recover action
	struct timespec tRecovery;
	//! Reading
	bool bReading;
	//! Mutex to manage the reads and writes
	pthread_mutex_t mutexSerial;
	//! Module ID and version of the board
	int iModuleID, iVersion;
	//! Counts the number of reading errors
	int num_of_reading_errors;
public:

	//! Public constructor
	rly08(const char *device, int baud_rate, const char *parity, int datasize);
	rly08();
	rly08(const char *device, double hz);
	//! Public destructor
	~rly08();
	//! Starts the control thread of the component and its subcomponents
	//! @return OK
	//! @return ERROR starting the thread
	//! @return RUNNING if it's already running
	//! @return NOT_INITIALIZED if it's not initialized
	virtual ReturnValue Start();


	//! Switch ON iRelay
    void RelayOn(int iRelay);
    //! Switch OFF iRelay
    void RelayOff(int iRelay);
    //! Gets Version and ModuleID
    void GetSwVersion(int *iMod_id,int *iVer);
    //! Get Relays sStatus
    unsigned int GetRelayStatus();

private:
	//! Actions in initial state
	virtual void InitState();
	//! Actions in Ready state
	virtual void ReadyState();
	//! Actions in Failure State
	virtual void FailureState();
	//! Actions in standby state
	virtual void StandbyState();
	//! Reads the status
	int ReadRelayStatus();
	//! Gets Version and ModuleID
    void ReadSwVersion();

};

#endif
