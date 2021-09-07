# Basic introduction examples for KDL and Eigen

AUT-720 Advanced Robotics 2021-2022 Course Material, example application for KDL and Eigen.

## Install backward_ros
Backward_ros is a package that can generate more detailed traceback to the error in C++ codes. If there is a mistake in the code, say an Eigen Matrix size error, normally the traceback will be very cryptic (mostly segment error or aborted) and won't give information about what specific line is causing an error. By using backward_ros, the error will be printed to the console screen with the code and the methods. By default, this package already loads the backward_ros package. Below, you can find the instructions to add backward_ros to other packages, such as ElfinSimulations/arm_controllers.

CHANGE `<ros-distro>` WITH YOUR DISTRO, SUCH AS `ros-noetic-backward-ros` (Ubuntu 20) OR `ros-melodic-backward-ros` (Ubuntu 18).

    $ sudo apt update
    $ sudo apt install ros-<ros-distro>-backward-ros

## Compile

    $ mkdir -p catkin_ws/src
    $ cd ~/catkin_ws/src
    $ git clone https://github.com/mehmetkillioglu/kdl_eigen_example.git
    $ cd ~/catkin_ws/
    $ catkin_make
    $ source devel/setup.bash

## Run

    $ rosrun kdl_eigen_example forward_kinematics



# [OPTIONAL] Adding backward_ros to other projects 

## Open CMakeLists.txt file

Find the  function. 

```
find_package(catkin REQUIRED COMPONENTS
<dependencies>
)
```

The debug type should be changed. Add `set(CMAKE_BUILD_TYPE RelWithDebInfo)` and add `backward_ros` below the dependencies. After the modification, the beginning of the `CMakeLists.txt` file should look like below where `<dependencies>` are the existing package dependencies (DO NOT DIRECTLY COPY PASTE THIS! ONLY ADD THE SPECIFIC LINES AS BELOW)

```
cmake_minimum_required(VERSION 2.8.3)
project(arm_controllers)

set(CMAKE_BUILD_TYPE RelWithDebInfo)       # ADD THIS LINE

find_package(catkin REQUIRED COMPONENTS
<dependencies>
backward_ros                               # ADD THIS LINE
)

```


## Open package.xml file

For ROS Noetic, add the following line under the lines with `<build_depend>pkg</build_depend>` as

```
<run_depend>backward_ros</run_depend>
```
Once you compile the package, the backward_ros should be initialized and the tracebacks will be printed to the console once an error occurs.

Haven't tested with the Ros Melodic, but same way should work just fine. If not, try as

```
<depend>backward_ros</depend>
```

## [OPTIONAL] See how the backward_ros works and how to interpret;

Without modifications, the example code should run fine that gives the positions of the end effector etc. To create an error, you can modify one of the eigen matrices to have smaller size than required. For instance, the `q_` vector normally have the same size as the number of joints. Find the line

```cpp
	q_.data = Eigen::VectorXd::Zero(n_joints_);
```
and modify it as 

```cpp
	q_.data = Eigen::VectorXd::Zero(n_joints_-1);
```

so the vector will have a smaller number than the required. If you run directly the code without backward_ros, you will get an error like 

```
forward_kinematics: /usr/include/eigen3/Eigen/src/Core/DenseCoeffsBase.h:425: Eigen::DenseCoeffsBase<Derived, 1>::Scalar& Eigen::DenseCoeffsBase<Derived, 1>::operator()(Eigen::Index) [with Derived = Eigen::Matrix<double, -1, 1>; Eigen::DenseCoeffsBase<Derived, 1>::Scalar = double; Eigen::Index = long int]: Assertion `index >= 0 && index < size()' failed.
Aborted (core dumped)

```

which does not contain any information about the where the error occurs specifically. It is obvious that an Eigen matrix has a wrong size, but no idea about which one exactly. However, if you add the backward_ros, the traceback will be more detailed as given below.

```

forward_kinematics: /usr/include/eigen3/Eigen/src/Core/DenseCoeffsBase.h:425: Eigen::DenseCoeffsBase<Derived, 1>::Scalar& Eigen::DenseCoeffsBase<Derived, 1>::operator()(Eigen::Index) [with Derived = Eigen::Matrix<double, -1, 1>; Eigen::DenseCoeffsBase<Derived, 1>::Scalar = double; Eigen::Index = long int]: Assertion `index >= 0 && index < size()' failed.
Stack trace (most recent call last):
#8    Object "", at 0xffffffffffffffff, in 
#7    Source "/home/mehmet/catkin_ws/src/kdl_eigen_example/src/forward_kinematics.cpp", line 292, in _start [0x55d77d6cd4dd]
        291:   return 0;
      > 292: }
#6    Source "../csu/libc-start.c", line 308, in __libc_start_main [0x7f3383b3e0b2]
#5    Source "/home/mehmet/catkin_ws/src/kdl_eigen_example/src/forward_kinematics.cpp", line 135, in main [0x55d77d6cb9d6]
        132:     q_(2) = 0;
        133:     q_(3) = 0;
        134:     q_(4) = 0;
      > 135:     q_(5) = 0;
        136:     std::cout << std::fixed << std::setprecision(4); // For clean printout
        137:     std::cout << "-----------------------------------------------" << std::endl;
        138:     // We can get the state of the end link using the forward kinematics solver
#4    Object "/usr/lib/liborocos-kdl.so.1.4.0", at 0x7f338406149e, in KDL::JntArray::operator()(unsigned int, unsigned int)
#3    Source "/build/glibc-eX1tMB/glibc-2.31/assert/assert.c", line 101, in __assert_fail [0x7f3383b4df35]
#2    Source "/build/glibc-eX1tMB/glibc-2.31/assert/assert.c", line 92, in __assert_fail_base [0x7f3383b3c728]
#1    Source "/build/glibc-eX1tMB/glibc-2.31/stdlib/abort.c", line 79, in abort [0x7f3383b3c858]
#0    Source "../sysdeps/unix/sysv/linux/raise.c", line 51, in raise [0x7f3383b5d18b]
Aborted (Signal sent by tkill() 61964 1000)
Aborted (core dumped)

```
The traceback still contains the original error about C++, however you can see the Stack trace section added. From the line #5 (will change for each code/project), you can see that the error occured at the line `135` of the code, in the `main` function. Also, a short code block is given with the line marked with `>`. From this, you can understand that the error of the size from Eigen is actually caused by the `q_` vector when it tried to access the element in the 5th index. 

For this simple project, you can probably debug yourself. However, when the code gets complicated and you have many vectors/matrices, as well as matrix multiplications, debugging becomes more challenging. I personally recommend you to install `backward_ros` to your project for this course in the beginning.
