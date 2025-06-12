# Group 6 Final Project

Our final project is aimed towards helping connect pupper, or other robots, to humans through virtual reality. Over the summer one of our team members worked with robots sent into nuclear power plants to help find broken pipes or objects because humans can’t. We gathered inspiration from that, aiming towards making our pupper do things that humans may not be able to do. Consequently, our project makes pupper complete the maze through human instruction (orientation of their head) via a VR headset, through which the human can see what pupper sees in real time.

A detailed diagram, our demo video, and more information can be found in the slides: https://docs.google.com/presentation/d/1L7-6jZHsvztuqr2Qocot1aQ54M2IG5Kyx1ZuEtT6df4/edit?usp=sharing

Instructions to run the code:
1. Connect headset (IMU board) to a local Macbook using cables.
2. On the Macbook, run imu_reader.py and command_publisher.py (in two separate terminals).
3. On Pupper, run ./launch_vr.sh.
4. On Macbook, open Foxglove and pick the stereo image option.
5. Facetime a phone while screensharing the stereo camera feed, and place the phone in the VR headset for display.

Installation and dependencies:
- the setup in Lab 6: Computer Vision
- install foxglove application
- pip install pyserial

Code Documentation:
- imu_reader.sh (on Macbook): reads quaternion coordinates from the IMU board, and parses it into roll, pitch, yaw, which are then used to determine the correct pupper movements
- command_publisher.sh (on Macbook): establishes UDP connection with pupper to send the movement commands (linear and angular velocity), which are read from a file that imu_reader wrote the commands to
- launch_vs.sh (on Pupper): a launch file that launches ROS and runs the files listener.py and stereo_image_publisher.py
- listener.py (on Pupper): listens to the movement commands send from command_publisher.sh and turns them into ROS instructions to move pupper accordingly
- stereo_image_publisher.py (on Pupper): takes pupper's camera feed and split it into two circluar views for VR, which are then read by foxglove on the Macbook (similar to lab 6: vision lab).

Note: we didn't get a chance to push the code on pupper to github since they were returned after demo day.
