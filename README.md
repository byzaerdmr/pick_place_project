# 🦾 Franka Panda Teleoperation System - Pick and Place

![Python](https://img.shields.io/badge/Python-3.9%2B-blue)
![PyBullet](https://img.shields.io/badge/Physics-PyBullet-green)
![Docker](https://img.shields.io/badge/Container-Docker-blue)
![License](https://img.shields.io/badge/License-MIT-yellow)

## 📌 Project Overview
This project implements a high-fidelity **Teleoperation System** for a 7-DOF Franka Panda robot arm within the PyBullet physics engine. The primary goal is to demonstrate precise object manipulation (pick-and-place) using a custom **Inverse Kinematics (IK)** controller and a modular software architecture.

Unlike standard scripts, this project is built upon **SOLID software engineering principles**, ensuring readability, modularity, and ease of maintenance.

## 🎯 Problem Statement & Goal
* **Problem:** Controlling a 7-DOF robot arm in Cartesian space is complex due to kinematic redundancy and singularity issues.
* **Goal:** To develop a robust simulation environment where a user can intuitively control the robot's End-Effector (x, y, z) and Gripper using keyboard inputs, facilitated by a real-time Jacobian-based IK solver.

---

## 🚀 Installation & Launch

### Prerequisites
* Python 3.9 or higher
* PIP (Python Package Manager)

### Method 1: Local Execution (Recommended)
1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/byzaerdmr/pick_place_project.git](https://github.com/byzaerdmr/pick_place_project.git)
    cd pick_place_project
    ```
2.  **Install dependencies:**
    ```bash
    pip install -r requirements.txt
    ```
3.  **Run the simulation:**
    ```bash
    python src/main.py
    ```

### Method 2: Docker (Containerization)
1.  **Build the image:**
    ```bash
    docker build -t panda-sim .
    ```
2.  **Run the container:**
    *(Note: Requires X11 forwarding for GUI visualization)*
    ```bash
    docker run -it --rm --net=host --env="DISPLAY" panda-sim
    ```

---

## 🎮 Implemented Features & Controls

The simulation operates in **Manual Teleoperation Mode**. The user acts as the high-level planner, while the IK controller handles joint calculations.

| Key | Action | Description |
| :--- | :--- | :--- |
| **Arrow Keys** (↑ ↓ ← →) | Move X / Y | Moves the end-effector in the horizontal plane. |
| **PgUp / PgDn** | Move Z Axis | Moves the end-effector Up and Down. |
| **Home / End** | Gripper Control | **Opens** (Home) or **Closes** (End) the fingers. |
| **Space** | Camera Switch | Cycles between **Main View**, **Side View**, and **Eye-in-Hand (FPS)**. |

### 📸 Example of Operation
Demonstration of Pick and Place Task

![Demo](docs/project.gif)

---

## 🛠️ Technical Architecture (SOLID Principles)
This project strictly follows software design principles to satisfy the course requirements:

1.  **Modularity:** The codebase is split into logical packages (`control`, `simulation`, `tasks`) rather than a monolithic script.
2.  **Single Responsibility Principle (SRP):**
    * `CameraManager`: Handles *only* view switching and matrix updates.
    * `IKController`: Handles *only* mathematical kinematic calculations.
    * `GripperController`: Handles *only* low-level motor control for fingers.
    * `World`: Manages *only* the environment setup (loading URDFs).
3.  **DRY (Don't Repeat Yourself):**
    * Common math utilities and workspace safety limits are centralized in `src/utils/safety.py`.
4.  **Dependency Injection:** Controllers accept the `robot_id` and `world` instances as arguments, reducing coupling.

---

## 📂 Repository Structure
```text
pick_place_project/
├── src/
│   ├── control/           # IK, Gripper, and Joint controllers
│   ├── simulation/        # World setup and PyBullet configuration
│   ├── tasks/             # Task logic definitions
│   ├── utils/             # Helper functions (safety limits, math)
│   └── main.py            # Main Event Loop & Entry Point
├── Dockerfile             # Container configuration for portability
├── requirements.txt       # Project dependencies
├── .gitignore             # Git exclusion rules
└── README.md              # Documentation
```

---

## 👥 Team & Roles

| Member | Key Contributions |
| :--- | :--- |
| **Beyza Erdemir - 500484**<br>(`byzaerdmr`) | **Simulation Logic:** Configured the PyBullet physics environment, implemented Inverse Kinematics (IK) algorithms, and robot controls. |
| **Murat Tekin Ulunisan - 500485**<br>(`muatr`) | **System Architecture:** Designed the modular code structure, implemented the Camera/Input systems, and managed Docker integration. |

---

## 📄 License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 📺 Project Defense & Presentation
Watch our project presentation:
[**Click Here to Watch the Video**](https://drive.google.com/file/d/1FhS6P76kyW6xx-_Zlk8q-66jUa9cnHZn/view)

## 📄 Presentation Slides
You can view the presentation slides in the `docs/` folder or [click here](docs/presentation.pdf) as PDF and [click here](docs/presentation.pptx) as PPTX.
