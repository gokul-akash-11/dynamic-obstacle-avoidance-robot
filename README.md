# Dynamic Obstacle Avoidance with Intent Prediction — TurtleBot3 (ROS 2 Jazzy + Gazebo Harmonic)

**Author:** Gokul Akash S  
**ROS Version:** ROS 2 Jazzy  
**Simulator:** Gazebo Harmonic (`gz sim`)  
**Robot:** TurtleBot3 Waffle Pi

This project implements a **dynamic obstacle–aware navigation system** using:

- YOLO–based perception 
- Kalman Filter–based motion tracking 
- JSON-based future path prediction 
- A Dynamic Planner that modifies Nav2 goals based on obstacle intent 
- A Goal Bridge that injects new goals into Navigation2 

Works with **real moving obstacles OR simulated fake obstacles**.

---

## 📌 Features

- Real-time YOLO object detection (camera → bounding boxes) 
- Kalman Filter tracking (smoothed center + short-horizon future points) 
- JSON-based intent publication (`/predicted_paths`) 
- Dynamic Planner: publishes alternate Nav2 goals when a predicted path is near the robot 
- Goal Bridge: forwards `/goal_pose` to Nav2's `NavigateToPose` action 
- Demo-friendly: fake-json mover for fast reproducible demos + optional Gazebo mover

---

## 🧩 System architecture

```
Camera (Gazebo)
     |
YOLO Detector
     |
Tracker (KF)
     |
Predicted Paths (JSON) <--- (optional fake_mover publishes here)
     |
Dynamic Planner
     |
Goal Bridge
     |
Nav2 (NavigateToPose)
     |
TurtleBot3 movement
```

---

## 📂 Workspace structure

```
ros2_ws/
└── src/
    ├── dynamic_planner/
    │   ├── dynamic_planner/
    │   │   ├── dynamic_planner_node.py
    │   │   └── goal_bridge.py
    │   ├── scripts/
    │   │   ├── fake_mover.py
    │   │   └── tb3_obstacle_mover.py
    │   ├── package.xml
    │   ├── setup.py
    │   └── setup.cfg
    │
    ├── tracker_kf/
    │   ├── tracker_kf/
    │   │   └── tracker_kf_node.py
    │   ├── package.xml
    │   ├── setup.py
    │   └── setup.cfg
    │
    └── yolo_detector/
        ├── yolo_detector/
        │   └── yolo_node.py
        ├── package.xml
        ├── setup.py
        └── setup.cfg
```

---

## 🚀 How It Works

### **1. YOLO Detector**
Reads camera feed → publishes detected bounding boxes.

### **2. Tracker KF**
Uses Kalman Filter to estimate:
- Object velocity 
- Smoothed position 
- Short-horizon future positions 

Publishes:
```json
{
"1": { "label": "person", "future": [[x1,y1],[x2,y2],...] }
}
```

### **3. Dynamic Planner**
Reads prediction → decides:
- If collision likely 
- If detour required 
- Generates a shifted Nav2 goal 

### **4. Goal Bridge**
Takes new goal → converts → sends to Nav2 Action Server.

### **5. TurtleBot3 Navigation**
Robot moves using Nav2 while avoiding dynamic obstacles.

---

## ⚠️ Known Limitations

- Requires YOLO to see the obstacle (field-of-view dependent) 
- Prediction horizon is short (KF limitation) 
- Nav2 replanning not instantaneous under heavy CPU load 
- At present, the project includes a lightweight JSON/fake-mover method as the stable demo path while Gazebo dynamic-world integration is being matured

---

## 🧭 Future Improvements

- LSTM-based long-range intent prediction 
- Multi-robot dynamic avoidance 
- Integration with RMF crowd simulation 
- Stereo depth integration 

---

## © License & copyright

© 2025 Gokul Akash S.

**No open-source license applied. All rights reserved.**  

This repository is publicly visible **for viewing only**.  
No permission is granted to copy, modify, distribute, or reuse any part of this project without explicit written permission from the author.

---

## 🙌 Acknowledgements

- Open-source TurtleBot3 community 
- Gazebo Harmonic + ROS-GZ bridge 
- Nav2 & ROS 2 contributors 
