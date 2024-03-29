# How to control the Galen robot with CRTK compatible commands

This README covers from creating your own account and moving the Galen robot.


Author: Hisashi Ishida (hishida3@jhu.edu)


## Create User account
Please make sure to create your own account (standard), not admin user.
You can create an account through `Settings` or the follwoing command from the sudo user account. 

```
sudo adduser <username>
```

## Clone CISST and CRTK repo 
Please follow the instructions on the JHU-Install.md (https://bitbucket.org/GalenRobotics/researchrepo/src/jhu-modern-cisst/JHU-Install.md) in the "jhu-modern-cisst" branch.

### Source correct files
Please source the correct folder to achieve system-wide availability of ROS and CRTK:
```
source /opt/ros/noetic/setup.bash 
source ~/catkin_ws/devel/setup.bash 
```

Or you can also permanently add the install location in your .bashrc with the following command:
```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash " >> ~/.bashrc
```


## Clone Galen code from the bitbucket
```
git clone git@bitbucket.org:GalenRobotics/researchrepo.git
cd researchrepo
git checkout jhu-modern-cisst # Use jhu-modern-cisst branch
```

## Build the Galen code 
Follow the instructions on the bitbucket "README.md" (https://bitbucket.org/GalenRobotics/researchrepo/src/master/).
We are only building REMS, but if you want to build Clinical-GUI, repeat the command with "-DTARGET=CGUI."


You can change the option "-j4" according to the number of cores that your cpu has. 
```
mkdir build && cd build
cmake ../source -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-DREGULAR_BUILD -DENGINEERING_BUILD" -DTARGET=REMS 
make -j4
```


## How to run Galen in Research mode
### Run Galen robot
Open a terminal and command `roscore`.

```
roscore
```
Open another and run the follwoing command.
```
cd build/devel/lib/REMS
./REMS
```

Once the REMS GUI is open, press `Robot Off` and change the mode to `Research`.
You can check the rostopics by `rostopic list` and `rostopic echo /REMS/Research/measured_cp`.

## Clone this repository
You can clone this repository by the following command:
```
git clone git@github.com:LCSR-CIIS/Galen_CRTK_example.git
cd Galen_CRTk_example
```


You can run the scipt with a namespace as a option. Ex) /REMS/Research
```
cd script
python3 galen_crtk_move_cp_example.py <namespace>
```
Press `m` and Press `Enter` to execute the drilling example.
