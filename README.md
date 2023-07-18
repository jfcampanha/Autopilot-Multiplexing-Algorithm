# autopilot_multiplexing
Linux Environment Aplication!
-----------------------------

Install libuavcan and libfort libraries, check if Socket CAN is in the Linux system where the programs will run

Go to the build folder, and run:
	cd build
	~/vcan.sh     (if in user's home directory)
	
Run both the publisher and subscriber codes, with respective Node IDs, like in the example:
	./subscriber_sensors 50
	./publisher_sensors 51 52 53
