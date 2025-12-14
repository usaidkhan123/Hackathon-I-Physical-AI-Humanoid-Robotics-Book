<!--
---
Sync Impact Report
---
- **Version Change**: `0.0.0` → `1.0.0`
- **Summary**: Initial establishment of the Supreme Project Constitution.
- **Principles Added**:
  - I. PURPOSE OF THE CONSTITUTION
  - II. SCOPE DOCTRINE
  - III. TARGET AUDIENCE MANDATE
  - IV. PEDAGOGY LAW
  - V. TECHNOLOGY STACK RESTRICTION
  - VI. RAG & CHATBOT SAFETY CONSTITUTION
  - VII. SPEC-DRIVEN DEVELOPMENT WORKFLOW RULES
  - VIII. QUALITY & COMPLETENESS REQUIREMENTS
  - IX. DEVELOPER TRANSPARENCY RULES
  - X. GOVERNANCE & AMENDMENT PROCEDURE
  - XI. RATIFICATION
- **Templates Requiring Updates**:
  - `spec-template.md`: Pending validation
  - `plan-template.md`: Pending validation
  - `tasks-template.md`: Pending validation
-->

# Supreme Project Constitution: “Teaching Physical AI & Humanoid Robotics: A Hands-On Guide to Embodied Intelligence.”

## I. PURPOSE OF THE CONSTITUTION

This Constitution exists to establish supreme, unambiguous, and enforceable governance over all content, workflows, and generated materials for the project. It ensures every component—from text and diagrams to AI-driven generation—adheres to a unified standard of quality, scope, and pedagogical integrity. It explicitly prohibits the creation or inclusion of any content that falls outside its defined doctrines. This document is the single source of truth for project rules.

## II. SCOPE DOCTRINE (MANDATORY)

The book, all associated materials, and all generated content MUST remain strictly limited to the following modules and topics. Any content outside this scope is unconstitutional unless explicitly added by a formal amendment.

-   **Module 1: The Robotic Nervous System (ROS 2)**
    -   ROS 2 Nodes, Topics, Services, and Actions
    -   `rclpy` programming for robotic behaviors
    -   URDF for humanoid robot structure definition
-   **Module 2: The Digital Twin (Gazebo & Unity)**
    -   Physics-based simulation environments
    -   Simulation of sensors including LiDAR, Depth Cameras, and IMUs
    -   Design of human-robot interaction scenarios
-   **Module 3: The AI-Robot Brain (NVIDIA Isaac)**
    -   Isaac Sim for high-fidelity robotics simulation
    -   Isaac ROS for Visual SLAM (VSLAM)
    -   Nav2 for humanoid path planning and navigation
-   **Module 4: Vision-Language-Action (VLA)**
    -   Whisper for voice-based human-robot interface
    -   LLM-driven planning from natural language to ROS Actions
    -   Development of a capstone autonomous humanoid system

## III. TARGET AUDIENCE MANDATE

All content, examples, and instructional materials MUST be engineered to serve beginners and intermediate learners. Content designed for advanced, graduate-level, or deeply academic audiences is prohibited unless it is fundamentally re-contextualized and simplified for a beginner-first audience. Complexity must be introduced incrementally and with sufficient scaffolding.

## IV. PEDAGOGY LAW (NON-NEGOTIABLE TEACHING STYLE)

The book and all its components MUST adhere to the following pedagogical principles:

-   **Beginner-First**: Assume no prior expertise.
-   **Clarity and Simplicity**: Use simple, direct language. Paragraphs MUST be short and focused.
-   **Hands-On Before Theory**: Introduce practical, hands-on demonstrations before delving into theoretical concepts.
-   **Analogies as Standard**: Every complex concept MUST be explained with a relatable analogy.
-   **Diagrams for Systems**: Every system involving multiple components MUST be illustrated with a clear diagram.

Furthermore, each chapter MUST conclude with the following four sections, in order:
1.  A set of hands-on exercises.
2.  A mini robotics integration task.
3.  A 5–10 question quiz to test understanding.
4.  A single, standalone coding assignment.

## V. TECHNOLOGY STACK RESTRICTION

The project, including all examples, documentation, and the RAG (Retrieval-Augmented Generation) system, MUST exclusively use the following technologies. The use of any other tool, framework, SDK, or language is prohibited unless approved via constitutional amendment.

-   **Documentation**: Docusaurus
-   **AI Generation**: Gemini CLI + SpecKit
-   **Backend**: FastAPI (Python)
-   **Vector DB**: Qdrant Cloud
-   **Metadata**: Neon Postgres
-   **Chatbot**: OpenAI Agents + ChatKit
-   **Hardware**:
    -   NVIDIA Jetson Orin Nano/NX
    -   Intel RealSense D435i
    -   IMU sensors
    -   Unitree Go2 or G1 (optional)
    -   Digital Twin Workstation (NVIDIA RTX GPU mandatory)

## VI. RAG & CHATBOT SAFETY CONSTITUTION

The integrated RAG assistant MUST operate under the following strict behavioral and architectural rules:

-   **Allowed Behavior**:
    -   It MUST only provide answers derived directly from indexed book content.
    -   It MUST always display the retrieved text snippets used to generate an answer.
    -   It MUST cite the specific chapters and sections for all information provided.
    -   It MUST prioritize the safety of the user and the hardware in all responses.
-   **Forbidden Behavior**:
    -   It is PROHIBITED from hallucinating, inventing, or inferring content not present in the source material.
    -   It is PROHIBITED from providing instructions that could lead to dangerous robotic operations.
    -   It is PROHIBITED from making claims about hardware performance or capabilities not explicitly stated in the book.
    -   It is PROHIBITED from producing speculative or unverified engineering guidance.
-   **Required Architecture**:
    -   The system MUST be implemented as a multi-agent architecture comprising, at minimum: a Retriever Agent, a Book Reasoner Agent, a Citation Agent, and a Safety Guard Agent.

## VII. SPEC-DRIVEN DEVELOPMENT WORKFLOW RULES

All book content—including text, code, diagrams, and examples—MUST be generated using the official Spec-Driven Development workflow provided by SpecKit and the Gemini CLI. The mandatory workflow is as follows:

1.  `/sp.plan`: Define the chapter's structure and plan.
2.  `/sp.expand`: Expand planned sections into detailed content.
3.  `/sp.implement`: Generate code, diagrams, and practical examples.
4.  `/sp.summarize`: Create the chapter summary.

Any content produced outside of this prescribed pipeline is invalid and unconstitutional. All prompts and interactions with the agent MUST adhere to the standards defined in the `.specify/templates/` directory.

 
Every chapter MUST, without exception, include the following components to be considered complete and constitutional:

-   At least one system diagram.
-   At least one complete, real-world code example.
-   At least one robotics mini-project.
-   A "Test Your Understanding" quiz.
-   A "Hands-On Assignment" section.
-   Explicit safety notes pertaining to robot hardware control and operation.

## IX. DEVELOPER TRANSPARENCY RULES

To ensure the book serves as a practical reference for developers building similar systems, the constitution MUST enforce complete transparency in its technical explanations. This includes providing:

-   Full source code for FastAPI application examples.
-   Detailed Qdrant vector database schemas.
-   Complete Neon Postgres table designs.
-   Clear diagrams of the RAG and multi-agent architecture.
-   In-depth explanations of the embedding, chunking, and retrieval strategies employed.

## X. GOVERNANCE & AMENDMENT PROCEDURE

This Constitution is the supreme law of the project. Amendments are permitted but MUST follow a strict procedure:

-   **Versioning**: All changes to this Constitution will be tracked using Semantic Versioning (MAJOR.MINOR.PATCH).
    -   **MAJOR**: Backward-incompatible changes, removal of principles, or fundamental shifts in governance.
    -   **MINOR**: Addition of new principles or significant expansion of existing sections.
    -   **PATCH**: Minor clarifications, typo corrections, or non-substantive wording changes.
-   **Amendment Process**: A change proposal must be submitted as a pull request with a detailed rationale. Approval requires a majority vote from the project's designated core contributors.
-   **Protected Sections**: The "Purpose of the Constitution" (Section I) and "Governance & Amendment Procedure" (Section X) are protected and require unanimous approval for any modification.

## XI. RATIFICATION

This section formalizes the adoption and version of this Constitution.

**Version**: 1.0.0
**Ratified**: 2025-12-07
**Last Amended**: 2025-12-07