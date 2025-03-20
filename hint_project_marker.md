# ROS vizualization hint

If you want to vizualize roll and pitch data in rviz2 you may use Marker message type together with broadcasting transformation frame in `/tf` topic. 

An example how to do that may be found in the `rollpitch_simulator.py` file.
Run it with `ros2 run bip_package tf_simulator`.

In `rviz2` you need to add Marker display, with `/visualization_marker` topic and fixed frame set to `world`. You may also add Axes display with `board` reference frame to see the frame attached to the object.

Finally you should see the image similar to the one below, animated according to random roll and pitch angles. 

![Screenshot of rviz2](./marker.png)