# Extending GLIM


## Notation

In this package, the (SE3) transformation from frame B to frame A is denoted as ```T_A_B```. In other words, a point ```p_B``` in the frame B is transformed into the frame A by ```T_A_B``` (i.e., ```p_A = T_A_B * p_B```).

For instance, the transformation from a LiDAR frame to the world frame (i.e., a LiDAR pose in the world frame) is represented as ```T_world_lidar```, and a point in the LiDAR frame ```p_lidar``` is transformed into the world frame by ```p_world = T_world_lidar * p_lidar```.

Similarty, ```v_A_B``` represents the velocity of frame B in frame A, and thus ```v_world_imu``` represents an IMU velocity in the world frame.

## Variables

### Variable addressing in GTSAM

In GTSAM, variables can be addressed using the ***gtsam::Symbol*** class that combines a symbol label (a character, e.g., 'x') and a variable index. For example, you can insert a pose variable into ***gtsam::Values*** and label it as ```X(i)``` as follows:

```cpp
using gtsam::symbol_shorthand::X;

int id = 0;
gtsam::Pose3 pose;

gtsam::Values values;
values.insert(X(id), pose);
```

!!! info
    See **5.2. Keys and Symbols** in [this tutorial](https://gtsam.org/tutorials/intro.html) for more details.

### Variables in GLIM

In the following, we show the graph structures and variables in the odometry estimation and global optimization algorithms of GLIM. Through the global callback slot mechanism, which will be explained later, you can access variables in the factor graphs and create additional constraints (i.e., factors) to improve the accuracy/stability/robustness of the mapping process in specific situations.

#### Variables in the odometry estimation

- ```X(i)``` : IMU pose = T_odom_imu (gtsam::Pose3)
- ```V(i)``` : IMU velocity = v_odom_imu (gtsam::Vector3)
- ```B(i)``` : IMU bias (gtsam::imuBias::ConstantBias)

!!! note
    In the odometry estimation, old variables are eliminated from the graph when they leave the sliding optimization window specified by the **smoother_lag** param (e.g., 5 sec). You thus need to ensure that additional factors refer to only variables in this optimization window. 

!!! note
    Because **odometry_ct** performs LiDAR-only estimation, it does not create ```V(i)``` and ```B(i)```, and ```X(i)``` represents the LiDAR pose instead of the IMU pose.

!!! info
    ["velocity_suppressor.cpp"](https://github.com/koide3/glim_ext/blob/master/modules/odometry/velocity_suppressor/src/glim_ext/velocity_suppressor.cpp) in **gtsam_points** shows a simple example to insert velocity suppression factors into the odometry estimation factor graph.

#### Variables in the global optimization

**Sub mapping states**:

- ```X(i)``` : Sensor pose = T_odom_sensor (gtsam::Pose3)
- ```V(i)``` : IMU velocity = v_odom_sensor (gtsam::Vector3)
- ```B(i)``` : IMU bias (gtsam::imuBias::ConstantBias)

**Global mapping states**:

- ```X(i)``` : Submap pose = T_world_submap (gtsam::Pose3)
- ```V(2 * i) & V(2 * i + 1)``` : IMU velocity at endpoints (gtsam::Vector3)
- ```B(2 * i) & V(2 * i + 1)``` : IMU bias at endpoints (gtsam::Vector3)


## Global callback slot

The global callback slot is a mechanism to hook processing steps in the mapping system. It enables to access the internal states of the mapping process and insert additional factors into the factor graph. The following code demonstrates how we can register a callback function to the new frame creation event in the odometry estimation and retrieve estimated sensor states of the latest frame.


```cpp

#include <glim/odometry/callback.hpp>

using namespace glim;

void on_new_frame(const EstimationFrame::ConstPtr& new_frame) {
  const long id = new_frame->id;                                    // Frame ID
  const double stamp = new_frame->stamp;                            // Timestamp
  const Eigen::Isometry3d& T_world_imu = new_frame->T_world_imu;    // IMU pose
}

void setup_callback() {
  using std::placeholders::_1;
  OdometryEstimationCallback::on_new_frame.add(std::bind(&on_new_frame, _1));
}
```

Take a look at the following links to see what can be retrieved from and inserted into the system via global callback slots:

* [OdometryEstimationCallbacks](glim/structglim_1_1OdometryEstimationCallbacks.html)
* [SubMappingCallbacks](glim/structglim_1_1SubMappingCallbacks.html)
* [GlobalMappingCallbacks](glim/structglim_1_1GlobalMappingCallbacks.html)

!!! warning
    Each of odometry estimation, submapping, and global mapping modules are run in different threads, and thus their callbacks can be called from different threads. **You must take care of the thread-safety** of your extension module if it subscribes to events from different modules.


## Extension module

GLIM offers a mechanism to load extension modules from shared libraries at run-time. To implement an extension module, you need to write your extension module class that inherits from ***glim::ExtensionModule*** and define a loading function named ***create_extension_module()*** that returns an instance of your extension class.


```cpp title="my_extension_module.cpp"
#include <glim/odometry/callbacks.hpp>
#include <glim/util/extension_module.hpp>

using namespace glim;

class MyExtensionModule : public ExtensionModule {
public:
  MyExtensionModule() {
    using std::placeholders::_1;
    OdometryEstimationCallbacks::on_new_frame.add(std::bind(&MyExtensionModule::on_new_frame, this, _1));
  }

  void on_new_frame(const EstimationFrame::ConstPtr& frame) {
    // ...
  }
};

extern "C" ExtensionModule* create_extension_module() {
  return new MyExtensionModule();
}
```


```cmake title="CMakeLists.txt"
cmake_minimum_required(VERSION 3.5.2)
project(my_extension_module)

set(CMAKE_CXX_STANDARD 17)

find_package(glim REQUIRED)

add_library(my_extension_module SHARED
  src/my_extension_module.cpp
)
target_link_libraries(my_extension_module
  glim::glim
)
```

The extension module can be loaded into GLIM by adding the name of the created shared library to ```extension_modules``` parameter in ```glim/config/config_ros.json```.

!!! note
    If you don't want to use the dynamic loading mechanism, it is also possible to create an extension by directly modifying ```glim_ros.cpp```, of course.
