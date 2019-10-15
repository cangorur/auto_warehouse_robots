cd ${HOME}/catkin_ws/

echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

rosdep update 

rosdep fix-permissions 
sleep 5
rosdep install --from-paths src --ignore-src --rosdistro=${ROS_DISTRO} -y 

cd factory_morse 
morse import auto_smart_factory
cd ${HOME}/catkin_ws/

echo "source /opt/ros/${ROS_DISTRO}/setup.bash"; catkin_make
sleep 5
echo "source ${HOME}/catkin_ws/devel/setup.bash" >> ~/.bashrc
sleep 1
echo "export PYTHONPATH=$PYTHONPATH:/usr/local/lib/python3/dist-packages/" >> ~/.bashrc
sleep 1
echo "source ~/.bashrc"

