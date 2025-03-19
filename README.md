![QR code to this page](qr-code.png)

# Getting started

## Linux basics

To start a new terminal: `Ctrl-Alt-t`

To inspect history of previous commands: `up/down-arrows`

To (reverse-)search a command in the history: `Ctrl-r` *text_to_search*

To complete commands/file/topic names etc.: press `Tab` after first letters (`Tab Tab` will present hints on available options)

## Virtual machine

1. Run the VM manager - start `VirtualBox`
2. Start the `Ros2Humble` VM
3. When the Ubuntu starting screen appears, login as 
    user: ros      password: ros2user


Note: Some commands require installation of additional packages with 
`sudo apt install ros-humble-rqt-graph ros-humble-rqt-plot ros-humble-rqt-reconfigure; pip install pyserial`

## First steps in ROS

1. Run two terminals (one for a publisher and one for a subscriber)
2. In the publisher terminal run
    `ros2 run demo_nodes_py talker`
3. In the subscriber node verify the messages are sent
    - With a subscriber `ros2 run demo_nodes_py listener`
    - With basic ROS commands
        - `ros2 topic list`
        - `ros2 topic hz /chatter`
        - `ros2 topic echo /chatter`
4. Verify the structure with ROS basic commands - in the 3rd terminal, having one of the previous subscribers running
    - List all topics: `ros2 node list --all`
    - Check the ROS graph: `ros2 run rqt_graph rqt_graph`


# ROS communication

## Creating the ROS packages

1. Have two terminals open: one for performing the actual tasks (T1) and another for building the package (T2) 
2. In T2 
    - stay in the main workspace folder (`*_ws`)
    - to rebuild packages after creation/changes - run `colcon build`
    - note: usually you will want to speed up the process by adding `--packages-select` parameter to the build command, so it will become `colcon build --packages-select *your_package_name*`
    - to refresh the setup after build run `source install/setup.bash`
    - note: you may combine the two commands as `colcon build --packages-select bip_package; source install/setup.bash`
3. To create a new python package  
    - Go to the source subfolder of the workspace `cd src` (in T1)
    - Create a new package `ros2 pkg create --build-type ament_python --node-name my_talker bip_package` (in T1)
    - Build and refresh setup in T2
    - Run the created node `ros2 run bip_package my_talker`
4. To get familiar with the required elements of the package, check the files in `src` folder
    - Package description in `package.xml`
    - Python instalation data in `setup.cfg` 
    - Python setup `setup.py` (note the `entry_points`, as you may want to update it when adding more scripts to the package)
    - Finally - the main script in `bip_package/my_talker.py`

## The first publisher

Using the ROS 2 tutorial we will create a basic publisher sending string messages.

1. Open the https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html webpage to create a publisher sending String messages
2. Follow the steps in the **section 2** of the tutorial, note the modifications to the package files
3. Use the topic name `/chatter` instead of `/topic` from the tutorial
4. When ready, build and run; use the subscriber from *First steps..* to verify

## Simulated acceleration data publisher

Now the publisher will be modified, to send the random data in `AccelStamped` format.

1. Go to the `src/bip_package/bip_package`; Copy the string publisher string to the new name
2. Add the new script to package configuration files; check it builds and runs
3. Modify the script to send the randomized acceleration data
    - Add proper message format to the imports
    - Modify the initialization of the publisher: topic name and format
    - Correct the data sent `timer_callback()`
    - (note: you may leave message empty first to check if it is published correctly - build and run, verify with echo)
    - Use python `random` to fill the message with random values
    - Build and run
4. Verify the publisher works correctly
    - using tools from *First steps...*
    - note: check values of the fileds sent, including the header
    - plotting graph with rqt_plot
        - run `ros2 run rqt_plot rqt_plot` 
        - select topic and add fields to inspect
        - note: in `rqt_plot` the fields are separated with "`/`", not "`.`"
        - note: to get correct plot, the timestamp field of the message should be set; to fill it with the current time you may use `msg.header.stamp = self.get_clock().now().to_msg()` (where self is the class derived from `Node`)
    - vizualizing with rviz
        - run `rviz2` 
        - use add - by topic - AccelStamped
        - note: if you do not see the result, check plot tree for errors and inspect fields of the message
        - note: `frame_id` field of the header should contain a name (string) of a coordinate frame (you may use any now)
5. (bonus) Add dynamic parameters to the script
    - Add to the import section

        `from rcl_interfaces.msg import ParameterDescriptor`
    - Add the parameter in the initialization of the node

        `scale_descriptor = ParameterDescriptor(description='Scaling coefficient')

        self.declare_parameter('scale', 1.0, scale_descriptor)`
    - Add reading of the parameter when preparing a new message for publish

        `scale_param = self.get_parameter('scale').get_parameter_value().double_value`
    - Add reaction to the parameter to data processing
    - Verify your script reacts to the paramters
        - changing the parameter value in commanf line

            `ros2 param set /acc_publisher scale *value*`
        - from GUI 

            `ros2 run rqt_reconfigure rqt_reconfigure`

            note: for the first run you may need to run it with option `--force-discover`


## Real acceleration data publisher - serial port 

The next task is to replace the simulated data with the real measurements read from the serial port.

Upload to the ESP32 module your code sending accelerometer data.

1. Connect the board to the USB port
2. Enable the access to the USB device from the VM 
    - mark the proper checkbox in Devices - USB
    - check the board is visible, for example by `ls /dev/ttyUSB* /dev/ttyACM*`
    - verify the communication is working by running `screen *device* *baudrate*`, for example `screen /dev/ttyUSB0 19200`
3. Create a new script or modify the string publisher
    - read the text received on the serial port and publish it to the ros `String`-type topic
    - verify the messages are passed correctly
4. Create a new script or modify the simulator
    - read the message from the serial port, transform it to 3 parameter vector and publish it as the `AccelStamped` message
    - verify the publisher with `rviz2` and/or `rqt_plot`
    - note: you may use regular expressions (`re`) select data from the string, but it may be easier to modify the ESP32 code to send data in simplified form (i.e. comma separated numbers)


# Bonus tasks

1. Modify the arduino program and your receiver to communicate with binary messages instead of plain text; try to answer the questions below:
    - What may be a benefit of that approach?
    - What are the drawback?
2. Modify both programs to accomodate other communication types (depending of your hardware capabilities)
    - Bluetooth (you need to enable the device for VM)
    - UDP over WiFi (you may create a hotspot from your laptop or ESP32)
3. Use the parameters to modify the settings of the connection and the transmission
    