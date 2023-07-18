#Autopilot Multiplexing Algorithm
---------------------------------
This repository represents an Autopilot Multiplexing Algorithm, based on a Publisher-Subscriber messaging patern of the DroneCAN protocol in a Linux environment.

**Install _libuavcan_ and _libfort_ libraries, also check if _SocketCAN_ is in the Linux system where the program will run.**

After cloning the repository go to the build folder:
-	cd build
Then, initialize the virtual CAN interface with _SocketCAN_ running the **vcan.sh** bash file:
-	~/vcan.sh     (if in user's home directory)
	
Run both the publisher and subscriber codes, with respective Node IDs, like in the example:
-	./subscriber_sensors 50
-	./publisher_sensors 51 52 53

After that, the application should run with no problems.
