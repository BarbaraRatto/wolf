HOW TO CREATE THE URDF, SRDF AND SRD FILES:
-------------------------------------------
First add the path of the packged to the ```ROS_PACKAGE_PATH``` env variable so that ROS can find it:

```export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:path/to/iit-teleop-ros-pkg```

If everything goes well you should be able to ```roscd teleop_urdf```.

Then go to the ```../iit-cogimon-ros-pkg/cogimon_urdf/script/``` folder and run:

```./create_urdf_srdf_sdf.sh teleop```

Due to missing packages there could be some error messages, the final ```.urdf```, ```.srdf``` and ```.sdf``` should 
anyway be created.
