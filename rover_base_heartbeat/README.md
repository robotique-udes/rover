# rover_base_heartbeat

Contains heartbeat_node node: 
    subscribed to : none

    publish to : heartbeat

Contains Services :
    /heartbeat_node/E_stop :
        bool E_Stop	#True : enable E_stop / False : disable E_stop
        ---
        bool result	#True : E_stop active / False : E_stop inactive

Notes:

The published message is irrelevent, the watchdog is only looking for the publish frequency.
Enabling the E_stop stops publishing the heartbeat message, wich triggers a warning in the watchdog.	
