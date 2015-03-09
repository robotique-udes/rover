/*
 * rly08_node
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
#include <rly_08/GetSwVersion.h>

#include <robotnik_msgs/inputs_outputs.h>
#include <robotnik_msgs/set_digital_output.h>


#include <self_test/self_test.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_updater/diagnostic_updater.h>
#include <diagnostic_updater/update_functions.h>
#include <diagnostic_updater/DiagnosticStatusWrapper.h>


class rly08Node
{
public:

	rly08 *rly_;
	
  	robotnik_msgs::inputs_outputs reading;
	self_test::TestRunner self_test_;
	diagnostic_updater::Updater diagnostic_;

	ros::NodeHandle	node_handle_;
	ros::NodeHandle	private_node_handle_;
	// services	
	ros::ServiceServer rly08_set_digital_outputs_;
	ros::ServiceServer rly08_get_sw_version_;
	// Topics
	ros::Publisher rly08_io_data_pub_;

	std::string sDevicePort;
	std::string sDefaultPort;	
	
	bool running;
	int	error_count_;
	int	slow_count_;
	std::string	was_slow_;
	double desired_freq_;
	diagnostic_updater::FrequencyStatus	freq_diag_;	
	int outputs_;	// Number of available digital outputs

	rly08Node(ros::NodeHandle h) : 
		self_test_(), 
		diagnostic_(), 
		node_handle_(h), 
		private_node_handle_("~"), 
		error_count_(0),
		slow_count_(0),
		desired_freq_(10),
		freq_diag_(diagnostic_updater::FrequencyStatusParam(&desired_freq_, &desired_freq_, 0.1))
		{
			running = false;
			ros::NodeHandle rly08_node_handle(node_handle_, "rly_08_node");
			
			// READING PARAMETERS
			sDefaultPort.assign(RLY08_DEFAULT_PORT);
			private_node_handle_.param("port", sDevicePort,  sDefaultPort);
			private_node_handle_.param("outputs", outputs_,  RLY08_DEFAULT_OUTPUTS);
			
		
			rly08_set_digital_outputs_ = rly08_node_handle.advertiseService("set_digital_outputs", &rly08Node::set_digital_outputs, this);
			rly08_get_sw_version_ = rly08_node_handle.advertiseService("get_sw_version", &rly08Node::get_sw_version, this);
			rly08_io_data_pub_ = rly08_node_handle.advertise<robotnik_msgs::inputs_outputs>("status", 100);
			self_test_.add("Connect Test", this, &rly08Node::ConnectTest);
			diagnostic_.add( freq_diag_ );
			diagnostic_.add( "Device Status", this, &rly08Node::deviceStatus );

			rly_ = new rly08(sDevicePort.c_str(), RLY08_THREAD_DESIRED_HZ);
			// Resizes the number of digital outputs
			reading.digital_outputs.resize(this->outputs_);
		}

	~rly08Node() { Stop(); }

	int Start()
	{
		int err_code = 0;

		err_code = Stop();
		if (err_code == ERROR) return err_code;

		
		err_code = rly_->Setup();
		if (err_code == ERROR) return err_code;

		// turns off all the outputs
		rly_->RelayOff(0);
		
		err_code = rly_->Start();
		if (err_code == ERROR) return err_code;


		running = true;
		return err_code;
	}

	int Stop()
	{
		if(running)
		{
			// turn off all the outputs
			rly_->RelayOff(0);
			rly_->ShutDown();
			running = false;
		}
		return 0;
	}

	bool spin()
	{
		ros::Rate r(10.0);		
		// Using ros::isShuttingDown to avoid restarting the node during a shutdown-
		while (!ros::isShuttingDown()) 
		{
			if (Start() == OK)
			{
				while(node_handle_.ok())
				{
					  if(read_and_publish() < 0) 
							break;
					  self_test_.checkTest();
					  diagnostic_.update();
					  ros::spinOnce();
					  r.sleep();
				}
			} 
			else 
			{
				// No need for diagnostic here since a broadcast occurs in start
				// when there is an error.
				usleep(1000000);
				self_test_.checkTest();
				ros::spinOnce();
			}
			
		}
		ROS_INFO("rly08Node::spin - calling stop !");
		Stop();
		return true;
	}

	int read_and_publish()
	{
		static double prevtime = 0;
		double starttime = ros::Time::now().toSec();

		if (prevtime && prevtime - starttime > 0.1)
		{
			ROS_WARN("rly08Node::read_and_publish: Full rly08 loop took %f ms. Nominal is 10ms.", 1000 * (prevtime - starttime));
			was_slow_ = "Full rly08 loop was slow.";
			slow_count_++;
		}

		
		int status = rly_->GetRelayStatus();
		
		for (int i=0; i<this->outputs_; i++)
		{
			reading.digital_outputs[i] = status & 1;
			status = status >> 1;
		}		

		double endtime = ros::Time::now().toSec();
		if (endtime - starttime > 0.1)
		{
			ROS_WARN("rly08Node::read_and_publish: Gathering data took %f ms. Nominal is 10ms.", 1000 * (endtime - starttime));
			was_slow_ = "Full rly08 loop was slow.";
			slow_count_++;
		}
		prevtime = starttime;
		starttime = ros::Time::now().toSec();
		rly08_io_data_pub_.publish(reading);

		endtime = ros::Time::now().toSec();
		if (endtime - starttime > 0.1)
		{
			ROS_WARN("rly08Node::read_and_publish: Publishing took %f ms. Nominal is 10 ms.", 1000 * (endtime - starttime));
			was_slow_ = "Full rly08 loop was slow.";
			slow_count_++;
		}

		freq_diag_.tick();
		return 0;
	}

	


	void ConnectTest(diagnostic_updater::DiagnosticStatusWrapper& status)
	{
		// connection test
		// TBC
		status.summary(0, "Connected successfully.");
	}

	void deviceStatus(diagnostic_updater::DiagnosticStatusWrapper &status)
	{
		if (!running)
			status.summary(2, "rly08 is stopped");
		else if(rly_->GetState() == FAILURE_STATE)
			status.summary(2, "rly08 is on FAILURE state");
			
		else if (!was_slow_.empty())
		{
			status.summary(1, "Excessive delay");
			was_slow_.clear();
		}
		else
			status.summary(0, "rly08 is running");

		status.add("Error count", error_count_);
		status.add("Excessive delay", slow_count_);
	}

	
	//! service to set the outputs value
	bool set_digital_outputs(robotnik_msgs::set_digital_output::Request &req,
		     robotnik_msgs::set_digital_output::Response &res)
	{
		int iRelay = req.output;
		int value = req.value;
		
		if (value != 0 && value != 1)
		{
			ROS_ERROR("Value must be 0 (off) or 1 (on).");
			res.ret = false;
		}
		else if (iRelay<0 || iRelay> this->outputs_)
		{
			ROS_ERROR("iRelay must be between 0 (all) and %d.", this->outputs_);
			res.ret = false;
		}
		else if (value == 0)
		{
			rly_->RelayOff(iRelay);
			res.ret = true;
		}	
		else
		{
			rly_->RelayOn(iRelay);
			res.ret = true;
		}	
		
		return true;
	}

	//! Gets and returns the controller sw version
	bool get_sw_version(rly_08::GetSwVersion::Request &req, rly_08::GetSwVersion::Response &res)
	{
		rly_->GetSwVersion(&res.module_id, &res.sw_version);
		return true;
	}


};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "rly08_node");

	ros::NodeHandle nh;
	rly08Node rlyN(nh);

	rlyN.spin();

	return 0;
}
