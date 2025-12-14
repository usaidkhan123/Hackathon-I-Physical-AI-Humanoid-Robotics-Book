---
id: 001
title: Create Specification for Robotics Curriculum
stage: spec
date: 2025-12-07
surface: agent
model: gemini-pro
feature: 001-robotics-curriculum-spec
branch: 001-robotics-curriculum-spec
user: User
command: /sp.specify
labels: [spec, curriculum, robotics]
links:
  spec: specs/001-robotics-curriculum-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-robotics-curriculum-spec/spec.md
  - specs/001-robotics-curriculum-spec/checklists/requirements.md
tests:
  - none
---

## Prompt

```
/sp.specify Create a complete and detailed specification for the educational book: Physical AI & Humanoid Robotics — The Embodied Intelligence Curriculum This specification must follow the constitution-based structure and define all modules, chapters, lessons, content rules, formatting rules, and Docusaurus output requirements. 1. Title Physical AI & Humanoid Robotics — A Complete Curriculum for Embodied Intelligence 2. Description Produce a full technical blueprint for a 12–14 week educational program teaching Physical AI, Humanoid Robotics, LLM-robot integration, NVIDIA Isaac, ROS 2, Gazebo, Unity, and Vision-Language-Action systems. The book teaches how to bridge digital AI models with physical robotic bodies, using simulation, robot middleware, and embodied interaction systems. 3. Book Structure Requirements 3.1 Module Structure The book contains 4 modules, each containing: 4 chapters Each chapter contains 3–4 lessons Increasing complexity across modules Hands-on activities + conceptual theory Real-world robotics examples 3.2 Modules Module 1 — The Robotic Nervous System (ROS 2) Module 2 — The Digital Twin (Gazebo & Unity) Module 3 — The AI-Robot Brain (NVIDIA Isaac) Module 4 — Vision-Language-Action Robotics (VLA) 4. Required Content for Each Module For every module define: Overview Learning outcomes Required software Required hardware Prerequisites Assessment criteria Hands-on implementation focus Real-world robotics connections Student deliverables 5. Chapter Structure Rules Each module contains 4 chapters, each chapter must include: 5.1 Chapter Format Chapter Title Chapter Summary Why this chapter matters Real Robotics Use-Cases Skills Students Will Build 5.2 Required 3–4 Lessons Per Chapter Each lesson must include: 6. Lesson Format (MANDATORY) Lesson Structure Each lesson must include: Lesson Title Lesson Objective (1 paragraph) Prerequisites Concept Explanation (detailed, multi-paragraph) Step-by-Step Technical Breakdown Real-World Analogy Hands-On Task (ROS, Gazebo, Isaac, Unity, Jetson, VLA depending on context) Code Example (Python + ROS 2 conventions) Common Mistakes & Debugging Tips Mini-Assessment (4–6 questions) 7. Docusaurus Requirements (MANDATORY) Generate a specification that supports building the book as a Docusaurus documentation site. 7.1 Folder Structure Define a consistent folder structure like: /docs /module-01-ros2 chapter-01.md chapter-02.md chapter-03.md chapter-04.md /module-02-gazebo-unity /module-03-nvidia-isaac /module-04-vla 7.2 Frontmatter for every lesson & chapter Every file must use Docusaurus YAML frontmatter: --- id: <unique-id> title: <chapter-or-lesson-title> sidebar_position: <number> description: <short description> --- 7.3 Markdown Requirements Use fenced code blocks Use images (placeholders allowed) Use callouts: Tip, Warning, Note 7.4 Search Optimization Include: Tags Keywords Section summaries Glossary definitions 8. Technical Depth Requirements The specification must include detailed coverage of: 8.1 ROS 2 Nodes, Topics, Services, Actions Launch files URDF for humanoids rclpy bridging Sensor integration 8.2 Gazebo & Unity Physics simulation SDF/URDF Sensor simulation Environment building Human-robot interaction visualization 8.3 NVIDIA Isaac Isaac Sim Isaac ROS VSLAM Navigation (Nav2) Reinforcement learning Sim-to-real transfer 8.4 Vision-Language-Action (VLA) Whisper speech input LLM task planning VLA pipelines Natural interaction design 9. Hardware Specification Requirements Include a detailed section describing: Digital Twin Workstation Jetson Orin Edge AI Kit Sensors (RealSense, IMU, LiDAR) Robot options (Unitree Go2, G1, proxy robots) Cloud simulation option (AWS G5/G6e nodes) 10. Final Deliverable of sp.specify The output MUST be a fully structured, extremely detailed book specification containing: ✔ All modules ✔ All chapters ✔ All lesson descriptions and content ✔ Docusaurus metadata ✔ Folder structures ✔ Formatting rules ✔ Content creation guidelines ✔ Glossary + reference material sections ✔ Assessment overview This specification should be long, exhaustive, highly structured, and provide a full blueprint to generate the entire book
```

## Response snapshot

I have successfully created the feature specification for the "Physical AI & Humanoid Robotics Curriculum".

1.  **Branch Created**: `001-robotics-curriculum-spec`
2.  **Specification Created**: The detailed specification is located at `specs/001-robotics-curriculum-spec/spec.md`.
3.  **Validation Checklist**: A quality checklist has been created and passed at `specs/001-robotics-curriculum-spec/checklists/requirements.md`.

The specification is now ready for the planning phase (`/sp.plan`).

## Outcome

- ✅ Impact: High. Created the foundational blueprint for the entire book project.
- 🧪 Tests: N/A
- 📁 Files: 2 files created.
- 🔁 Next prompts: The user can now proceed to the planning phase with `/sp.plan`.
- 🧠 Reflection: The provided prompt was extremely detailed, which allowed for a high-quality specification to be generated without any need for clarification. The initial attempt to use the `create-new-feature.ps1` script failed due to ambiguity in its parameter handling, but falling back to manual git and file operations was a successful recovery strategy.

## Evaluation notes (flywheel)

- Failure modes observed: Script execution failure.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: N/A
