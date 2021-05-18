# rover_control

How to use watchdog:

	- watchdog.yaml:

		 nb_sensors: 6			: how many topic will be watched
			sensor_1:		
  			name: imu		: Name of sensor
  			sensor_id: imu		: Sensor id to differenciate similar sensor
  			topic: /imu/data	: Topic to look at
  			max_freq: 60		: Upper bound of topic publish frequency
  			min_freq: 40		: Lower bound of topic publish frequency

	- aggregator.yaml:

	make sure 'num_items' in aggregator.yaml match 'nb_sensors' in watchdog.yaml 

	- watchdog is launched by control.launch file 
	- Launch rqt_robot_monitor

topic: /watchdog/cmd_vel
