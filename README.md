
# ECHO

### **E**nvironmental **C**hange and **H**istory of **O**bjects

**Persistent 3D Semantic Object Memory & Environmental Change Detection for Mobile Robots**

ECHO is a robotics research project exploring how autonomous mobile robots can **perceive, remember, and understand changes in their environment over time**.

Rather than treating each sensor observation independently, ECHO aims to maintain a persistent 3D object-centric world model that recognizes previously seen objects, detects when they appear, disappear, or move, and distinguishes temporary occlusions from genuine changes.

The long-term goal is to combine **3D perception, persistent memory, dynamic-agent tracking, and motion prediction** to support more informed autonomous navigation.

## 🎥 Demo

![ECHO Warehouse Demo](assets/echo_demo.gif)

*Nova Carter navigating NVIDIA's warehouse with ROS 2 Jazzy and Nav2 in Isaac Sim.*

## 🛠️ Technology Stack

| Component | Technology |
| --- | --- |
| Simulation | NVIDIA Isaac Sim 6.2 |
| Robotics | Nova Carter, ROS 2 Jazzy, Nav2 |
| Perception | Python, OpenCV, SAM 2.1 |
| Deep Learning | PyTorch, CUDA |
| Localization | AMCL, LiDAR, Odometry, TF |
| Development | Ubuntu 24.04, Git |

## 🎯 Research Objectives

- **Persistent 3D Memory:** Maintain object identities, locations, and observation histories across visits.
- **Environmental Change Detection:** Identify added, removed, relocated, and unchanged objects.
- **Dynamic Scene Understanding:** Track moving robots and humans while reasoning about occlusion.
- **Motion & Route Analysis:** Assess whether environmental changes or moving agents may affect planned paths.
- **Navigation Integration:** Investigate object-aware obstacle updating and clearing for Nav2.

## 🚀 Development Milestones

| Milestone | Status |
| --- | --- |
| 01 — Isaac Sim warehouse and Nova Carter setup | ✅ Complete |
| 02 — ROS 2 Nav2 autonomous navigation | ✅ Complete |
| 03 — Fixed-camera RGB-D perception and object association prototypes | ✅ Complete |
| 04 — Multi-robot warehouse and autonomous background agents | 🚧 **In Progress** |
| 05 — Moving-robot 3D perception and localization | ⏳ Planned |
| 06 — Persistent object memory and revisit-aware change detection | ⏳ Planned |
| 07 — Dynamic-agent tracking and motion prediction | ⏳ Planned |
| 08 — Route-impact analysis and Nav2 integration | ⏳ Planned |
| 09 — Quantitative evaluation and research paper | ⏳ Planned |

**📍 Current Focus — Milestone 04**

An additional Nova Carter has spawned and moved autonomously using NVIDIA Replicator Agent. Static-obstacle avoidance, sensor visibility, and more complex multi-agent scenarios are still being validated.

## ▶️ Getting Started

**1. Start Isaac Sim**

```bash
~/echo/scripts/start_echo_sim.sh
```

**2. Start Nav2 and RViz**

```bash
~/echo/scripts/start_echo_nav.sh
```

Use **Nav2 Goal** in RViz to send an autonomous navigation goal.

These commands launch the verified single-robot navigation baseline. Dynamic-agent experiments use separate scenes and configurations.

## 🔬 Research & Evaluation

ECHO will be evaluated using controlled environmental changes, repeated robot visits, and randomized dynamic-agent scenarios.

Key metrics include:

- 3D object localization error
- Object association accuracy
- Change-detection precision and recall
- Tracking and motion-prediction errors
- Obstacle-clearing latency
- Navigation and route-impact performance

---

*ECHO is a research prototype under active development.*

**Verified navigation baseline:** `echo-nav-v1`
