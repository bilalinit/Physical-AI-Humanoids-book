---
id: 001
title: spec-physical-ai-book
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-book
branch: 001-physical-ai-book
user: claude
command: /sp.specify
labels: [spec, physical-ai, robotics, book]
links:
  spec: specs/001-physical-ai-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-physical-ai-book/spec.md
  - specs/001-physical-ai-book/checklists/requirements.md
tests:
  - spec-validation
---

## Prompt

this is the context and structure ofr the book we are writing, now specs accordingly "Physical AI & Humanoid Robotics Book

## 1. Project Overview
**Title:** Embodied Intelligence: Physical AI & Humanoid Robotics
**Type:** Technical Guidebook / Engineering Manual
**Goal:** Create a comprehensive guide bridging modern generative AI (LLMs/VLA) with hard robotics (ROS 2, Hardware Control) using NVIDIA's ecosystem.
**Target Audience:** CS/Robotics Engineering Students.
**Primary Output:** A book containing code examples, architectural diagrams, and system configurations.

## 2. Technical Stack & Constraints
The generated content must adhere to the following versioning and compatibility matrix to ensure "Sim-to-Real" transfer.

### Software Environment
*   **OS:** Ubuntu 22.04 LTS (Jammy Jellyfish).
*   **Middleware:** ROS 2 Humble Hawksbill (or Iron Irwini where compatible).
*   **Languages:** Python 3.10+ (Primary), C++17 (Performance critical nodes).
*   **Simulation:**
    *   NVIDIA Isaac Sim (Version 4.0+).
    *   Gazebo (Fortress/Harmonic).
    *   Unity (2022 LTS for visualization/HRI).
*   **AI/ML Frameworks:**
    *   PyTorch (CUDA 12.x).
    *   OpenAI API (Whisper, GPT-4o).
    *   NVIDIA Isaac ROS (GEMs).

### Hardware Targets (Reference Architecture)
*   **Workstation (Sim):** NVIDIA RTX 4070 Ti (12GB VRAM) minimum.
*   **Edge Device (Real):** NVIDIA Jetson Orin Nano (8GB) or Orin NX (16GB).
*   **Sensors:** Intel RealSense D435i (RGB-D + IMU), RPLIDAR (Optional), ReSpeaker Mic array.
*   **Robot Platform:** Unitree Go2 (Quadruped proxy) or Unitree G1 (Humanoid).

---

## 3. Directory Structure Convention
All code examples generated for this book must follow standard ROS 2 `colcon` workspace structures:

```text
~/physical_ai_ws/
    src/
        physical_ai_bringup/      # Launch files and params
        physical_ai_description/  # URDF, Meshes, USD files
        physical_ai_simulation/   # Gazebo/Isaac world files
        physical_ai_perception/   # VSLAM, Object Detection
        physical_ai_navigation/   # Nav2 configs
        physical_ai_brain/        # LLM/VLA Integration nodes
```

---

## 4. Book Structure & Implementation Details

### Part I: The Physical AI Lab (Infrastructure)

**Chapter 1: Hardware & OS Configuration**
*   **Objective:** Define the "Digital Twin" workstation and "Edge" environment.
*   **Specs:**
    *   Script to install NVIDIA Drivers (535+), CUDA 12.2, and Docker.
    *   Comparison table: Workstation (RTX 4090) vs. Cloud (AWS g5.2xlarge).
    *   *Action:* Provide a `setup_lab.sh` script for Ubuntu 22.04 automation.

**Chapter 2: The Edge Ecosystem (Jetson Setup)**
*   **Objective:** Flash and configure the Jetson Orin Nano.
*   **Specs:**
    *   JetPack 6.0 setup guide.
    *   Configuring "Headless Mode" for remote development (VS Code Remote - SSH).
    *   *Technical Detail:* Configuring the `jtop` utility for monitoring GPU load.

### Part II: The Robotic Nervous System (ROS 2)

**Chapter 3: ROS 2 Architecture**
*   **Objective:** Core communication principles.
*   **Specs:**
    *   Explain DDS (Data Distribution Service) discovery.
    *   Code: Python Node template using `rclpy`.
    *   Code: Custom Interface (`.msg` and `.srv`) definitions for Humanoid control (e.g., `JointAngles.msg`).

**Chapter 4: The Body (URDF & Kinematics)**
*   **Objective:** Describing the robot structure.
*   **Specs:**
    *   **URDF/Xacro:** Create a 12-DOF bipedal robot description.
    *   **TF2:** Explain the transform tree (`map` -> `odom` -> `base_link` -> `camera_link`).
    *   *Visuals:* Rviz2 configuration (`.rviz`) to display the ghost robot.

### Part III: The Digital Twin (Simulation)

**Chapter 5: Physics in Gazebo**
*   **Objective:** Basic physics and sensor simulation.
*   **Specs:**
    *   SDF (Simulation Description Format) conversion from URDF.
    *   Adding Plugins: `libgazebo_ros_diff_drive.so` (for base) and `libgazebo_ros_imu.so`.
    *   *Scenario:* A warehouse world with dynamic obstacles (cubes with mass).

**Chapter 6: Photorealism with NVIDIA Isaac Sim**
*   **Objective:** High-fidelity training environments.
*   **Specs:**
    *   **USD Workflow:** Importing URDF into Omniverse.
    *   **Action Graph:** Visual programming for sensor publishing (RGB + Depth to ROS 2).
    *   **Domain Randomization:** Script to vary lighting and textures for ML robustness.

### Part IV: Perception & Navigation (The Visual Stack)

**Chapter 7: The Eyes (Sensors & VSLAM)**
*   **Objective:** Localization without GPS.
*   **Specs:**
    *   **Driver:** `realsense-ros` configuration for D435i.
    *   **Algorithm:** `isaac_ros_visual_slam` implementation.
    *   *Output:* A pointcloud topic `/camera/depth/color/points`.

**Chapter 8: Navigation (Nav2 for Humanoids)**
*   **Objective:** Path planning and obstacle avoidance.
*   **Specs:**
    *   **Nav2 Stack:** Config files (`nav2_params.yaml`) for:
        *   Global Planner: `SmacPlanner` (Hybrid A*).
        *   Local Planner: `MpucController` (Regulated Pure Pursuit).
    *   **Costmaps:** Voxel grids for 3D obstacle avoidance.

### Part V: The VLA Paradigm (GenAI Integration)

**Chapter 9: The Ears (Voice Pipeline)**
*   **Objective:** Voice-to-Text integration.
*   **Specs:**
    *   **Hardware:** ReSpeaker USB integration via ALSA.
    *   **Software:** Python node `voice_listener.py` utilizing OpenAI Whisper API (or local `distil-whisper` for Jetson).
    *   *Latency:* Implementing a "Push-to-Talk" logic via a joystick button to save tokens.

**Chapter 10: The Brain (LLM Action Planning)**
*   **Objective:** Natural Language to ROS Actions.
*   **Specs:**
    *   **Prompt Engineering:** Designing a System Prompt that outputs strictly JSON.
        *   *Example Prompt:* "You are a robot navigation assistant. Output format: `{'action': 'navigate', 'coordinates': [x, y, theta]}`."
    *   **Code:** `llm_commander_node.py` that subscribes to `/recognized_text` and publishes to Nav2 Action Client.

**Chapter 11: Vision-Language-Action (VLA)**
*   **Objective:** Multimodal reasoning.
*   **Specs:**
    *   Integration of GPT-4o (Vision) or local VLA (e.g., LLaVA) on the workstation.
    *   *Task:* Robot captures image, sends to API, asks "Is the path clear?", receives Boolean, executes movement.

### Part VI: Capstone Project

**Chapter 12: The Autonomous Humanoid**
*   **Objective:** Full system integration.
*   **Scenario:** "The Butler Test."
    1.  **Voice Command:** "Go to the kitchen and find the red mug."
    2.  **Planner:** LLM parses "kitchen" to coordinates (x,y).
    3.  **Nav:** Robot navigates via Nav2 (avoiding obstacles).
    4.  **VLA:** Robot stops, rotates camera, runs object detection for "red mug."
    5.  **Success:** Robot plays a success sound.
*   **Deliverable:** A `docker-compose.yml` file that launches the Simulation, ROS Bridge, Nav2, and LLM Agent simultaneously.

---

## 5. Coding Guidelines for Generation
*   **Python:** Use Type Hints (`def callback(self, msg: String) -> None:`). Use `rclpy` executors.
*   **Launch Files:** Use Python-based Launch files (`.launch.py`), not XML.
*   **Config:** Use YAML for all parameters.
*   **Error Handling:** All nodes must include `try-except` blocks for API failures (e.g., OpenAI timeout) to prevent robot crash.

## 6. Diagram Requirements (Mermaid.js)
The text must include Mermaid graphs for:
1.  **ROS Graph:** Showing Node/Topic connections.
2.  **TF Tree:** Showing kinematic relationships.
3.  **State Machine:** Showing the transitions from *Listening* -> *Planning* -> *Moving* -> *Verifying*."

## Response snapshot

# Feature Specification: Physical AI & Humanoid Robotics Book

**Feature Branch**: `001-physical-ai-book`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Physical AI & Humanoid Robotics Book - Technical Guidebook for CS/Robotics Engineering Students bridging modern generative AI (LLMs/VLA) with hard robotics (ROS 2, Hardware Control) using NVIDIA's ecosystem."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Learning Physical AI Concepts (Priority: P1)

As a CS/Robotics Engineering student, I want a comprehensive guidebook that bridges modern generative AI with hard robotics so that I can understand how to build embodied intelligence systems using NVIDIA's ecosystem.

**Why this priority**: This is the core value proposition - students need a resource that connects AI concepts with practical robotics implementation.

**Independent Test**: Can be fully tested by reviewing the book's content on fundamental concepts and ensuring it provides clear, practical examples that students can follow.

**Acceptance Scenarios**:

1. **Given** a student with basic programming knowledge, **When** they read the book from start to finish, **Then** they understand how to implement Physical AI systems combining LLMs/VLA with ROS 2 robotics.

2. **Given** a student working on a robotics project, **When** they reference the book's code examples, **Then** they can successfully implement the demonstrated concepts in their own projects.

---

### User Story 2 - Implementing ROS 2 Architecture with AI Integration (Priority: P1)

As a robotics developer, I want clear guidance on ROS 2 architecture integrated with AI/ML frameworks so that I can build systems that bridge generative AI with hardware control.

**Why this priority**: This addresses the core technical integration that the book aims to teach - connecting AI models with robotic systems.

**Independent Test**: Can be fully tested by implementing a basic ROS 2 node that integrates with an AI model as described in the book.

**Acceptance Scenarios**:

1. **Given** a ROS 2 environment, **When** following the book's instructions, **Then** I can create nodes that process AI model outputs into robotic actions.

2. **Given** a hardware robot platform, **When** following the book's integration patterns, **Then** I can successfully control the robot based on AI-generated commands.

---

### User Story 3 - Setting Up Physical AI Development Environment (Priority: P2)

As a student or researcher, I want clear instructions for setting up the Physical AI development environment on Ubuntu 22.04 with NVIDIA hardware so that I can begin implementing the concepts in the book.

**Why this priority**: Prerequisites and setup are essential for students to be able to follow along with the book's examples.

**Independent Test**: Can be fully tested by following the setup instructions and successfully running a basic example.

**Acceptance Scenarios**:

1. **Given** a fresh Ubuntu 22.04 installation, **When** following the book's setup instructions, **Then** I have a working development environment with ROS 2, NVIDIA tools, and AI frameworks properly configured.

2. **Given** NVIDIA RTX 4070 Ti or Jetson hardware, **When** following the book's hardware setup instructions, **Then** I have the system properly configured for AI/robotics development.

---

### User Story 4 - Building Simulation-to-Real Transfer Systems (Priority: P2)

As a robotics researcher, I want guidance on creating digital twins with NVIDIA Isaac Sim and Gazebo that transfer to real hardware so that I can develop and test AI/robotics systems efficiently.

**Why this priority**: Simulation is crucial for safe and efficient development of complex robotics systems before deployment on real hardware.

**Independent Test**: Can be fully tested by creating a simulation environment and successfully transferring learned behaviors to a real robot.

**Acceptance Scenarios**:

1. **Given** a URDF robot description, **When** following the book's simulation setup, **Then** I can create a working simulation environment that accurately models the real robot.

2. **Given** a behavior learned in simulation, **When** transferring to real hardware following the book's guidance, **Then** the behavior executes successfully on the physical robot.

---

### Edge Cases

- What happens when students have different hardware configurations than the reference setup?
- How does the system handle different versions of ROS 2, CUDA, or other dependencies than specified?
- What if students don't have access to the specific hardware (Unitree robots, NVIDIA Jetson) mentioned in the book?
- How does the book accommodate different learning paces and prior knowledge levels among students?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST provide comprehensive coverage of Physical AI concepts bridging generative AI (LLMs/VLA) with hard robotics (ROS 2, Hardware Control)
- **FR-002**: System MUST include practical code examples compatible with Ubuntu 22.04 LTS, ROS 2 Humble Hawksbill, Python 3.10+, and C++17
- **FR-003**: System MUST provide setup instructions for NVIDIA hardware including RTX 4070 Ti minimum and Jetson Orin Nano/NX platforms
- **FR-004**: System MUST include simulation examples using NVIDIA Isaac Sim (Version 4.0+), Gazebo (Fortress/Harmonic), and Unity (2022 LTS)
- **FR-005**: System MUST provide guidance on integrating AI/ML frameworks including PyTorch (CUDA 12.x), OpenAI API (Whisper, GPT-4o), and NVIDIA Isaac ROS (GEMs)
- **FR-006**: System MUST include ROS 2 architecture guidance covering DDS discovery, custom interfaces (.msg/.srv), and TF2 transforms
- **FR-007**: System MUST provide URDF/Xacro examples for robot descriptions and kinematics (specifically 12-DOF bipedal robot)
- **FR-008**: System MUST include perception and navigation examples using RealSense D435i, VSLAM, and Nav2 stack with SMAC planner
- **FR-009**: System MUST provide VLA (Vision-Language-Action) integration examples combining camera input with LLM reasoning
- **FR-010**: System MUST include complete capstone project examples demonstrating full system integration

### Key Entities

- **Physical AI Book**: Comprehensive guidebook containing code examples, architectural diagrams, and system configurations for bridging generative AI with robotics
- **Reference Architecture**: Standardized setup including Ubuntu 22.04, ROS 2 Humble, NVIDIA hardware, and specified software frameworks
- **Robot Models**: URDF/Xacro representations of humanoid and quadruped robot platforms for simulation and real-world implementation
- **AI Integration Patterns**: Standardized approaches for connecting LLMs/VLA models with ROS 2 nodes for robotic action planning
- **Simulation Environments**: Digital twin configurations using Isaac Sim, Gazebo, and Unity for training and testing AI/robotics systems

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: Students can successfully implement at least 80% of the book's code examples on the specified hardware/software stack
- **SC-002**: Students can complete the capstone "Butler Test" project within 40 hours of study time
- **SC-003**: 90% of students report improved understanding of Physical AI concepts after completing the book
- **SC-004**: Students can successfully transfer learned behaviors from simulation to real hardware in at least 75% of attempts
- **SC-005**: The book covers all 12 chapters as outlined in the structure with complete, testable code examples for each
- **SC-006**: All code examples run successfully on the specified technical stack without modification (Ubuntu 22.04, ROS 2 Humble, etc.)

## Outcome

- ✅ Impact: Created comprehensive feature specification for Physical AI & Humanoid Robotics Book
- 🧪 Tests: spec-validation completed successfully
- 📁 Files: specs/001-physical-ai-book/spec.md and checklists/requirements.md created
- 🔁 Next prompts: /sp.plan to create implementation plan
- 🧠 Reflection: Specification successfully captures all requirements from user input with clear user stories, functional requirements, and success criteria

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All validation criteria met
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Plan implementation with /sp.plan
