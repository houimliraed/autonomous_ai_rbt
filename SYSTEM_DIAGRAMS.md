# AUTONOMBOT - Visual System Diagrams

This document contains ASCII diagrams that can be converted to proper technical diagrams for presentations.

## System Architecture Overview

```
                    AUTONOMBOT COMPLETE SYSTEM ARCHITECTURE
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │                                HARDWARE LAYER                               │
    ├─────────────────────────────────────────────────────────────────────────────┤
    │                                                                             │
    │  ┌───────────────┐  ┌───────────────┐  ┌───────────────┐  ┌─────────────┐  │
    │  │   JETSON      │  │   HC-SR04     │  │   RPLiDAR     │  │ RealSense   │  │
    │  │   XAVIER      │  │  Ultrasonic   │  │      A1       │  │    D435i    │  │
    │  │     AGX       │  │   Sensor      │  │  Laser Scanner│  │  RGB-D Cam  │  │
    │  └───────────────┘  └───────────────┘  └───────────────┘  └─────────────┘  │
    │          │                  │                  │                  │         │
    └──────────┼──────────────────┼──────────────────┼──────────────────┼─────────┘
               │                  │                  │                  │
    ┌──────────┼──────────────────┼──────────────────┼──────────────────┼─────────┐
    │          │        COMMUNICATION & INTERFACE LAYER        │                  │
    ├──────────┼──────────────────┼──────────────────┼──────────────────┼─────────┤
    │          ▼                  ▼                  ▼                  ▼         │
    │    ┌──────────┐      ┌──────────┐      ┌──────────┐      ┌──────────┐      │
    │    │   GPIO   │      │   GPIO   │      │   USB    │      │   USB    │      │
    │    │Interface │      │Interface │      │Interface │      │Interface │      │
    │    └──────────┘      └──────────┘      └──────────┘      └──────────┘      │
    │          │                  │                  │                  │         │
    └──────────┼──────────────────┼──────────────────┼──────────────────┼─────────┘
               │                  │                  │                  │
    ┌──────────┼──────────────────┼──────────────────┼──────────────────┼─────────┐
    │          │                  │    ROS 2 LAYER   │                  │         │
    ├──────────┼──────────────────┼──────────────────┼──────────────────┼─────────┤
    │          ▼                  ▼                  ▼                  ▼         │
    │  /motor_commands    /ultrasonic_range        /scan         /camera/*        │
    │          │                  │                  │                  │         │
    │          │                  └─────────┬────────┼──────────────────┘         │
    │          │                            │        │                            │
    │          ▼                            ▼        ▼                            │
    │  ┌──────────────┐            ┌─────────────────────┐                       │
    │  │ CONTROLLER   │            │   LOCALIZATION      │                       │
    │  │   PACKAGE    │            │      PACKAGE        │                       │
    │  │              │            │                     │                       │
    │  │ ┌──────────┐ │            │ ┌─────────────────┐ │                       │
    │  │ │Diff Drive│ │            │ │      EKF        │ │                       │
    │  │ │Controller│ │◄───────────┤ │  Sensor Fusion  │ │                       │
    │  │ └──────────┘ │            │ └─────────────────┘ │                       │
    │  │              │            │          │          │                       │
    │  │ ┌──────────┐ │            │ ┌─────────────────┐ │                       │
    │  │ │Odometry  │ │            │ │      AMCL       │ │                       │
    │  │ │Publisher │ │────────────┤ │  Localization   │ │                       │
    │  │ └──────────┘ │            │ └─────────────────┘ │                       │
    │  └──────────────┘            └─────────────────────┘                       │
    │          │                            │                                    │
    │          │                            ▼                                    │
    │          │                   ┌─────────────────────┐                       │
    │          │                   │    NAVIGATION       │                       │
    │          │                   │      PACKAGE        │                       │
    │          │                   │                     │                       │
    │          │                   │ ┌─────────────────┐ │    ┌─────────────────┐│
    │          │                   │ │   Path Planner  │ │    │     VISION      ││
    │          │                   │ │    (NavFn)      │ │    │    PACKAGE      ││
    │          │                   │ └─────────────────┘ │    │                 ││
    │          │                   │          │          │    │ ┌─────────────┐ ││
    │          │                   │ ┌─────────────────┐ │    │ │    YOLO     │ ││
    │          │                   │ │   Controller    │ │    │ │  Detection  │ ││
    │          └───────────────────┤ │     (DWB)       │ │◄───┤ └─────────────┘ ││
    │                              │ └─────────────────┘ │    │                 ││
    │                              │          │          │    │ ┌─────────────┐ ││
    │                              │ ┌─────────────────┐ │    │ │  Obstacle   │ ││
    │                              │ │  Behavior Tree  │ │    │ │Avoidance    │ ││
    │                              │ │   Navigator     │ │    │ └─────────────┘ ││
    │                              │ └─────────────────┘ │    └─────────────────┘│
    │                              └─────────────────────┘                       │
    └─────────────────────────────────────────────────────────────────────────────┘
```

## Sensor Integration Flow

```
                            SENSOR DATA PROCESSING PIPELINE
    
    Physical World
           │
           ▼
    ┌─────────────────────────────────────────────────────────────────────────┐
    │                          SENSOR INPUTS                                 │
    └─────────────────────────────────────────────────────────────────────────┘
           │
           ▼
    ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐
    │   HC-SR04   │  │ RPLiDAR A1  │  │ RealSense   │  │   Wheel Encoders   │
    │ 0.02-4.0m   │  │ 360° Scan   │  │  RGB-D      │  │   L/R Wheels       │
    │   Range     │  │ 0.15-12m    │  │  Camera     │  │   Velocity Data    │
    └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘
           │                │                │                    │
           ▼                ▼                ▼                    ▼
    /ultrasonic_range    /scan      /camera/color/image_raw    /odom_noisy
           │                │                │                    │
           └────────────────┼────────────────┼────────────────────┘
                            │                │
                            ▼                ▼
                   ┌─────────────────┐ ┌─────────────────┐
                   │  LOCALIZATION   │ │     VISION      │
                   │    SUBSYSTEM    │ │   PROCESSING    │
                   │                 │ │                 │
                   │ ┌─────────────┐ │ │ ┌─────────────┐ │
                   │ │     EKF     │ │ │ │   YOLOv8    │ │
                   │ │   Filter    │ │ │ │  Detection  │ │
                   │ └─────────────┘ │ │ └─────────────┘ │
                   │        │        │ │        │        │
                   │ ┌─────────────┐ │ │ ┌─────────────┐ │
                   │ │    AMCL     │ │ │ │  Obstacle   │ │
                   │ │ Localizer   │ │ │ │Classifier   │ │
                   │ └─────────────┘ │ │ └─────────────┘ │
                   └─────────────────┘ └─────────────────┘
                            │                │
                            ▼                ▼
                   /odometry/filtered  /obstacle_detection
                            │                │
                            └────────┬───────┘
                                     │
                                     ▼
                            ┌─────────────────┐
                            │   NAVIGATION    │
                            │    PLANNING     │
                            │                 │
                            │ ┌─────────────┐ │
                            │ │   Costmap   │ │
                            │ │  Generation │ │
                            │ └─────────────┘ │
                            │        │        │
                            │ ┌─────────────┐ │
                            │ │Path Planning│ │
                            │ │ & Control   │ │
                            │ └─────────────┘ │
                            └─────────────────┘
                                     │
                                     ▼
                              /cmd_vel (Robot Motion)
```

## Localization System Detail

```
                         LOCALIZATION ARCHITECTURE DETAIL
    
    Sensor Inputs                    EKF State Fusion                Output
    ┌─────────────┐                 ┌─────────────────┐             ┌─────────────┐
    │   Wheel     │────────────────►│                 │             │             │
    │  Odometry   │  [F,F,F,F,F,F,  │                 │             │  Filtered   │
    │ /odom_noisy │   T,T,F,F,F,F,  │   Extended      │────────────►│  Odometry   │
    └─────────────┘   F,F,F]        │   Kalman        │             │/odom/       │
                                    │   Filter        │             │filtered     │
    ┌─────────────┐                 │                 │             └─────────────┘
    │ Ultrasonic  │────────────────►│  State Vector:  │
    │   Motion    │  [F,F,F,F,F,F,  │  [x,y,z,        │             ┌─────────────┐
    │/ultrasonic_ │   T,F,F,F,F,T,  │   roll,pitch,   │             │             │
    │motion       │   F,F,F]        │   yaw,          │────────────►│Transform    │
    └─────────────┘                 │   vx,vy,vz,     │             │ Tree        │
                                    │   vroll,vpitch, │             │ /tf         │
    ┌─────────────┐                 │   vyaw,         │             └─────────────┘
    │ Ultrasonic  │────────────────►│   ax,ay,az]     │
    │  Position   │  [T,F,F,F,F,F,  │                 │
    │Constraint   │   F,F,F,F,F,F,  │                 │
    │/ultrasonic_ │   F,F,F]        │                 │
    │position_... │                 │                 │
    └─────────────┘                 └─────────────────┘
                                             │
                                             ▼
                                    ┌─────────────────┐
    ┌─────────────┐                 │                 │             ┌─────────────┐
    │    Map      │────────────────►│      AMCL       │             │             │
    │  /map       │                 │  Adaptive Monte │────────────►│  Global     │
    └─────────────┘                 │  Carlo          │             │Localization │
                                    │  Localization   │             │   /tf       │
    ┌─────────────┐                 │                 │             │(map→odom)   │
    │  LiDAR      │────────────────►│ Particle Filter │             └─────────────┘
    │   Scan      │                 │ 500-2000        │
    │  /scan      │                 │ particles       │
    └─────────────┘                 └─────────────────┘

    Legend: [x,y,z,roll,pitch,yaw,vx,vy,vz,vroll,vpitch,vyaw,ax,ay,az]
            T = True (sensor data used), F = False (ignored)
```

## Navigation System Flow

```
                            NAVIGATION SYSTEM ARCHITECTURE
    
    Goal Input                      Planning Layer                    Control Output
    ┌─────────────┐                ┌─────────────────┐               ┌─────────────┐
    │             │                │                 │               │             │
    │ Goal Pose   │───────────────►│   BT Navigator  │               │  Velocity   │
    │/goal_pose   │                │                 │               │ Commands    │
    │             │                │ ┌─────────────┐ │               │  /cmd_vel   │
    └─────────────┘                │ │ Behavior    │ │               │             │
                                   │ │ Tree        │ │               └─────────────┘
    ┌─────────────┐                │ │ Logic       │ │                      │
    │             │                │ └─────────────┘ │                      │
    │    Map      │───────────────►│        │        │                      ▼
    │   /map      │                │        ▼        │               ┌─────────────┐
    │             │                │ ┌─────────────┐ │               │             │
    └─────────────┘                │ │   Planner   │ │               │   Robot     │
                                   │ │   Server    │ │               │   Motion    │
    ┌─────────────┐                │ │             │ │               │             │
    │             │                │ │ ┌─────────┐ │ │               │ ┌─────────┐ │
    │Robot Pose   │───────────────►│ │ │  NavFn  │ │ │               │ │Wheels   │ │
    │/odom/       │                │ │ │Planner  │ │ │               │ │L & R    │ │
    │filtered     │                │ │ └─────────┘ │ │               │ └─────────┘ │
    │             │                │ └─────────────┘ │               └─────────────┘
    └─────────────┘                │        │        │
                                   │        ▼        │
    ┌─────────────┐                │ ┌─────────────┐ │
    │             │                │ │ Controller  │ │
    │ Obstacles   │───────────────►│ │   Server    │ │
    │/scan        │                │ │             │ │
    │/ultrasonic_ │                │ │ ┌─────────┐ │ │
    │range        │                │ │ │   DWB   │ │ │
    │             │                │ │ │Local    │ │ │
    └─────────────┘                │ │ │Control  │ │ │
                                   │ │ └─────────┘ │ │
    ┌─────────────┐                │ └─────────────┘ │
    │             │                │        │        │
    │Vision       │───────────────►│        ▼        │
    │Obstacles    │                │ ┌─────────────┐ │
    │/obstacle_   │                │ │ Behavior    │ │
    │detection    │                │ │   Server    │ │
    │             │                │ │             │ │
    └─────────────┘                │ │ ┌─────────┐ │ │
                                   │ │ │Recovery │ │ │
                                   │ │ │Behaviors│ │ │
                                   │ │ └─────────┘ │ │
                                   │ └─────────────┘ │
                                   └─────────────────┘

                                    Costmap Layers
                            ┌─────────────────────────────┐
                            │                             │
                            │ ┌─────────┐ ┌─────────────┐ │
                            │ │ Static  │ │  Obstacle   │ │
                            │ │  Layer  │ │   Layer     │ │
                            │ │ (Map)   │ │ (Sensors)   │ │
                            │ └─────────┘ └─────────────┘ │
                            │           │                 │
                            │           ▼                 │
                            │ ┌─────────────────────────┐ │
                            │ │    Inflation Layer     │ │
                            │ │   (Safety Margins)     │ │
                            │ └─────────────────────────┘ │
                            │           │                 │
                            │           ▼                 │
                            │ ┌─────────────────────────┐ │
                            │ │   Combined Costmap     │ │
                            │ │  (Global & Local)      │ │
                            │ └─────────────────────────┘ │
                            └─────────────────────────────┘
```

## Hardware Deployment Architecture

```
                          JETSON XAVIER DEPLOYMENT LAYOUT
    
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │                           JETSON XAVIER AGX                                 │
    │                         Hardware Integration                                │
    ├─────────────────────────────────────────────────────────────────────────────┤
    │                                                                             │
    │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
    │ │   Power     │  │   GPIO      │  │  USB 3.0    │  │    Ethernet         │ │
    │ │ Management  │  │   Pins      │  │   Ports     │  │    Network          │ │
    │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
    │        │               │               │                     │             │
    │        │               │               │                     │             │
    │        ▼               ▼               ▼                     ▼             │
    │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
    │ │   Motor     │  │   HC-SR04   │  │  RPLiDAR    │  │  Remote Access      │ │
    │ │  Drivers    │  │ Ultrasonic  │  │     A1      │  │  SSH/VNC/ROS        │ │
    │ │ (PWM/GPIO)  │  │   Sensor    │  │             │  │                     │ │
    │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
    │        │               │               │                     │             │
    │        │               │               │                     │             │
    │        ▼               ▼               ▼                     ▼             │
    │ ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────┐ │
    │ │             │  │             │  │ RealSense   │  │                     │ │
    │ │Left Motor   │  │Range: 2cm-  │  │   D435i     │  │   Development       │ │
    │ │Right Motor  │  │       400cm │  │ RGB-D Cam   │  │     Tools           │ │
    │ │             │  │             │  │             │  │                     │ │
    │ └─────────────┘  └─────────────┘  └─────────────┘  └─────────────────────┘ │
    └─────────────────────────────────────────────────────────────────────────────┘
    
    Software Stack on Jetson Xavier:
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │                            SOFTWARE LAYERS                                 │
    ├─────────────────────────────────────────────────────────────────────────────┤
    │ Ubuntu 22.04 LTS                                                           │
    │ ├── ROS 2 Humble Hawksbill                                                 │
    │ │   ├── Autonombot Packages (9 packages)                                   │
    │ │   ├── Nav2 Navigation Stack                                              │
    │ │   ├── Robot Localization (EKF)                                           │
    │ │   └── SLAM Toolbox                                                       │
    │ ├── Computer Vision                                                        │
    │ │   ├── OpenCV 4.x                                                         │
    │ │   ├── YOLOv8 (Ultralytics)                                               │
    │ │   └── RealSense SDK                                                      │
    │ ├── Hardware Drivers                                                       │
    │ │   ├── Jetson.GPIO                                                        │
    │ │   ├── RPLiDAR SDK                                                        │
    │ │   └── Custom Firmware                                                    │
    │ └── System Services                                                        │
    │     ├── Hardware Validation                                                │
    │     ├── Diagnostic Monitoring                                              │
    │     └── Safety Systems                                                     │
    └─────────────────────────────────────────────────────────────────────────────┘

    Resource Allocation:
    ┌─────────────────────────────────────────────────────────────────────────────┐
    │ CPU Cores (8x ARM A78AE): 60% utilization                                  │
    │ GPU (512-core Volta): 40% utilization (YOLO inference)                     │
    │ Memory (32GB LPDDR4x): 4GB active usage                                    │
    │ Storage (32GB eMMC): 2GB for maps and logs                                 │
    │ Network: 50MB/s internal, 1MB/s external                                   │
    └─────────────────────────────────────────────────────────────────────────────┘
```
