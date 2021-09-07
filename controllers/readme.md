# TCP socket connection
This simple python scripts are to communicate with vehicles via a server. After starting server in a terminal, the servers starts to listen clients. The clients share data such as position, speed or acceleration. Thus, it is possible to share date inter clients. 
* Set `tcpip_simple_street_controller` for vehicle1 and add a second vehicle which uses the controller `tcpip_simple_street_controller2` on webots.
* Start `python3 server_tcpi_test.py` in a terminal. Then, server will start to listen the clients (vehicle1 and vehicle2)
* Run the simulation from webots
