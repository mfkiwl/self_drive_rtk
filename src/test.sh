#!/bin/bash

sudo -S chmod 777 /dev/ttyS1 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS2 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS3 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS4 << EOF
1
EOF

sudo -S chmod 777 /dev/ttyS5 << EOF
1
EOF


sudo -S chmod 777 /dev/ttyS6 << EOF
1
EOF


# source /opt/ros/melodic/setup.bash
# source /home/sweet/github_store/robot_nav/devel/setup.bash

cd /home/sweet/workspace/robot/src
gnome-terminal --tab -- python tcp.py
gnome-terminal --tab -- python moveBase.py
gnome-terminal --tab -- python pushImg.py
gnome-terminal --tab -- python ctrl.py
gnome-terminal --tab -- python main.py
gnome-terminal --tab -- python rtk.py




#echo "this is a test" >> /home/sweet/github_store/robot_nav/src/base_controller/launch/text.log

# roslaunch /home/sweet/github_store/robot_nav/src/base_controller/launch/base_controller.launch
# roslaunch /home/sweet/github_store/robot_nav/src/robot_arm_config/launch/demo.launch

exit 0
