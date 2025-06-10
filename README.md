# Group 6 Final Project

Our final project is aimed towards helping connect pupper, or other robots, to humans through virtual reality. Over the summer one of our team members worked with robots sent into nuclear power plants to help find broken pipes or objects because humans can’t. We gathered inspiration from that, aiming towards making our pupper do things that humans may not be able to do. Consequently, our project makes pupper complete the maze through human instruction (orientation of their head) via a VR headset, through which the human can see what pupper sees in real time.

A detailed diagram, our demo video, and more information can be found in the slides: https://docs.google.com/presentation/d/1L7-6jZHsvztuqr2Qocot1aQ54M2IG5Kyx1ZuEtT6df4/edit?usp=sharing

Instructions to run the code:
1. Connect headset (IMU board) to a local Macbook using cables.
2. On the Macbook, run imu_reader.py and command_publisher.py.
3. On Pupper, run listener.py and stereo_image_publisher.py, and launch ROS.
4. On Macbook, open Foxglove and pick the stereo image option.
5. Facetime a phone while screensharing the stereo camera feed, and place the phone in the VR headset for display.

Installation and dependencies:
- the setup in Lab 6: Computer Vision
- install foxglove application
- pip install pyserial

