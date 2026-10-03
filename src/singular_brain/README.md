# Singular Brain - Centralized Intelligence for Dynamic Swarm Navigation

This repository contains the `singular_brain` ROS 2 package, developed as the core perception and centralized intelligence module for a swarm of mobile robots operating in GPS-denied, dynamic industrial environments (e.g., warehouses, factories).

This project fulfills the core perception requirements of the **Inter IIT Tech Meet 13.0 Mid Prep (Bharat Forge)** problem statement.

## 🌟 Key Features
- **YOLOv5s Object Detection:** Identifies up to 80 unique COCO classes in real-time, easily exceeding the 10 unique object requirement.
- **3D Localization:** Projects 2D bounding boxes into true 3D space relative to the camera lens using synchronized depth maps and dynamic dimension scaling.
- **Static vs. Dynamic Identification:** Continuously computes 3D Euclidean velocity across frames. Objects moving faster than `0.15 m/s` are automatically flagged as `dynamic` (e.g., `person_dynamic`), while stationary objects are flagged as `static` (e.g., `chair_static`).
- **Robust Tracking:** Employs a custom bipartite greedy matching tracker to maintain stable IDs for objects across frames, preventing ghosting.
- **Bleeding-Edge Compatibility:** Built entirely without `cv_bridge` to prevent Numpy 2.x API crashes, and utilizes ONNX models to support OpenCV 5.

---

## 🛠️ System Requirements
- **OS:** Ubuntu 22.04
- **ROS 2:** Humble Hawksbill
- **Simulation:** Gazebo Classic (11.x)
- **Python:** 3.10+ with `numpy` and `opencv-python`

---

## 📦 Installation & Setup

1. **Clone the repository into your ROS 2 workspace:**
   ```bash
   cd ~/robot_ws/src
   # Ensure both factorybot (URDFs) and singular_brain are in this directory
   ```

2. **Install ROS 2 Dependencies & Gazebo Models:**
   Gazebo Classic requires standard models to be cached locally to prevent it from freezing on startup.
   ```bash
   cd ~/robot_ws
   rosdep install --from-paths src --ignore-src -r -y
   
   # Download standard Gazebo models to prevent freezing
   mkdir -p ~/.gazebo
   git clone https://github.com/osrf/gazebo_models.git ~/.gazebo/models
   ```

3. **Build the Workspace:**
   ```bash
   source /opt/ros/humble/setup.bash
   colcon build --packages-select singular_brain
   source install/setup.bash
   ```

*(Note: The YOLOv5s ONNX model is automatically included in `singular_brain/models/` and copied to the `share` directory during the build).*

---

## 🚀 Execution & Testing

To test the Singular Brain's perception stack, you need to launch the Gazebo simulation and spawn a robot (e.g., the AMR), then start the perception node.

### 1. Launch Simulation & Spawn Robot
Open a new terminal and run:
```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws

# Process the URDF (patched to use relative paths for ROS 2)
xacro src/factorybot/robot/amr.urdf.xacro robot_name:=amr_1 > amr_1.urdf

# Start Gazebo Classic with the factory world and exported models path
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/robot_ws/src/factorybot/warehouse/models
ros2 launch gazebo_ros gazebo.launch.py world:=$(ros2 pkg prefix factorybot --share)/warehouse/factory.world &

# Spawn the robot into Gazebo
ros2 run gazebo_ros spawn_entity.py -entity amr_1 -file amr_1.urdf -x 0 -y 0 -z 0.1
```

### 2. Run the Perception Node
In a second terminal, start the perception stack:
```bash
source /opt/ros/humble/setup.bash
cd ~/robot_ws
source install/setup.bash

ros2 launch singular_brain perception.launch.py namespace:=amr_1
```

### 3. Verify 3D Detections
In a third terminal, monitor the output of the Singular Brain. Place an object (like a person or car) in front of the robot in Gazebo and move it around to see the Static/Dynamic classification update in real-time!

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /amr_1/perception/detections_3d
```

---

## 🏗️ Architecture Notes
- **Message Synchronization:** The node uses `message_filters.ApproximateTimeSynchronizer` to strictly bind RGB, Depth, and CameraInfo frames together. This prevents severe 3D coordinate miscalculations when the robot turns quickly.
- **Swarm Scalability:** The node is heavily parameterized. By passing `namespace:=amr_2` in the launch file, you can instantly scale this perception node to as many robots as your compute hardware can handle, feeding data up to the Centralized Database.
