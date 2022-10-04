# rover_cam

Contains camera_filter node: 
	subscribed to : /camera1/image_raw

	publish to : image_filter/image			#message type : Image
	publish to : image_filter/image/compressed	#message type : CompressedImage

Contains Services :
	/camera_filter/fps_filter :
		float32 fps	#frame per seconds wanted (default 30)
		bool active	#True : enable filter / False : disable filter
		---
		bool result	#True : filter active / False : filter inactive

	/camera_filter/gray_filter :
		bool active	#True : enable filter / False : disable filter
		---
		bool result	#True : filter active / False : filter inactive

	/camera_filter/size_filter :
		float32 size	#resolution reduction wanted (1.0 : no reduction ; 0.5 : 50% reduction ; etc)
		bool active	#True : enable filter / False : disable filter
		---
		bool result	#True : filter active / False : filter inactive
		
 
## To install Hugin panorama

    $ sudo add-apt-repository ppa:ubuntuhandbook1/apps
    $ sudo apt update
    $ sudo apt install hugin
