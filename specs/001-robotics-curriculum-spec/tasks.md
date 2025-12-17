# Tasks: Physical AI & Humanoid Robotics Curriculum

**Input**: Design documents from `specs/001-robotics-curriculum-spec/`
**Prerequisites**: `plan.md` and `spec.md` are required. `research.md`, `data-model.md`, `contracts/openapi.json` were also utilized.

**Organization**: Tasks are grouped into phases following the Spec-Driven Development workflow.

---

## Phase 1: Docusaurus Setup Tasks

- [ ] T001 Initialize Docusaurus project in `physical-ai-book` directory:
    ```bash
    npm create docusaurus@latest physical-ai-book classic
    ```
- [ ] T002 Navigate into the project directory:
    ```bash
    cd physical-ai-book
    ```
- [ ] T003 Install TypeScript dependencies:
    ```bash
    npm install --save-dev typescript @docusaurus/module-type-aliases @docusaurus/tsconfig @types/react
    ```
- [ ] T004 Generate `tsconfig.json`:
    ```bash
    npx tsc --init
    ```
- [ ] T005 Convert `docusaurus.config.js` to `docusaurus.config.ts`
- [ ] T006 Convert `src/pages/index.js` to `src/pages/index.tsx`
- [ ] T007 Convert `src/theme/Root.js` to `src/theme/Root.tsx`
- [ ] T008 Configure `docusaurus.config.ts` with project metadata, presets, and theme. Use provided snippet in `plan.md`.
- [ ] T009 Configure `sidebars.ts` to automatically generate from the `docs/` directory structure. Use provided snippet in `plan.md`.
- [ ] T010 Create directory structure for modules, chapters, and lessons as defined in `plan.md`.
- [ ] T011 Create empty Markdown files for all modules, chapters, and lessons as defined in `plan.md`.
- [ ] T012 Enable versioning by creating version `1.0.0`:
    ```bash
    npm run docusaurus docs:version 1.0.0
    ```

---

## Phase 2: User Story 1 - Module 1: The Robotic Nervous System (ROS 2) (P1)

Each module will contain four chapters, and each chapter will contain 3 to 4 lessons, as defined in `plan.md` and `spec.md`.

**Story Goal**: As a learner, I want to understand and use the fundamentals of ROS 2 so that I can build the foundational software structure for a humanoid robot.
**Independent Test Criteria**: The learner can write, build, and run a set of ROS 2 nodes that communicate via topics, services, and actions.

- [ ] T020 [US1] Create `module-01-ros2.md` as the main module overview file for Module 1 in `docs/module-01-ros2/module-01-ros2.md` with appropriate Docusaurus frontmatter.
- [ ] T021 [US1] For each chapter in Module 1, create a chapter overview Markdown file (e.g., `chapter-01-foundations.md` in `docs/module-01-ros2/chapter-01-foundations/chapter-01-foundations.md`) including:
    - Chapter Title
    - Chapter Summary
    - "Why this chapter matters" section
    - "Real Robotics Use-Cases" section
    - "Skills Students Will Build" section
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T022 [US1] For each lesson in Module 1, create a lesson Markdown file (e.g., `lesson-01-what-is-physical-ai.md` in `docs/module-01-ros2/chapter-01-foundations/lesson-01-what-is-physical-ai.md`) including:
    - Lesson Title & Objective
    - Prerequisites
    - Concept Explanation
    - Step-by-Step Technical Breakdown
    - Real-World Analogy
    - Hands-On Task
    - Code Example (Python + ROS 2)
    - "Common Mistakes & Debugging Tips" section
    - Mini-Assessment (4-6 questions)
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T023 [US1] Implement ROS 2 nodes, topics, services, and actions described in content for Module 1.
- [ ] T024 [US1] Develop and integrate code examples (Python + ROS 2) for Module 1.
- [ ] T025 [US1] Verify that all code examples for Module 1 run successfully and demonstrate the learning objectives.
- [ ] T026 [US1] Create a test suite to validate the independent test criteria for Module 1: `docs/module-01-ros2/tests/test_ros2_communication.py`.

---

## Phase 3: User Story 2 - Module 2: The Digital Twin (Gazebo & Unity) (P2)

Each module will contain four chapters, and each chapter will contain 3 to 4 lessons, as defined in `plan.md` and `spec.md`.

**Story Goal**: As a learner, I want to create and interact with a simulated humanoid robot in a physics-based environment so that I can safely test and develop robotic behaviors without needing physical hardware.
**Independent Test Criteria**: The learner can import a URDF model into Gazebo or Unity, add simulated sensors, and control the robot's joints.

- [ ] T027 [US2] Create `module-02-simulation.md` as the main module overview file for Module 2 in `docs/module-02-simulation/module-02-simulation.md` with appropriate Docusaurus frontmatter.
- [ ] T028 [US2] For each chapter in Module 2, create a chapter overview Markdown file (e.g., `chapter-01-gazebo-basics.md` in `docs/module-02-simulation/chapter-01-gazebo-basics/chapter-01-gazebo-basics.md`) including:
    - Chapter Title
    - Chapter Summary
    - "Why this chapter matters" section
    - "Real Robotics Use-Cases" section
    - "Skills Students Will Build" section
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T029 [US2] For each lesson in Module 2, create a lesson Markdown file (e.g., `lesson-01-installation.md` in `docs/module-02-simulation/chapter-01-gazebo-basics/lesson-01-installation.md`) including:
    - Lesson Title & Objective
    - Prerequisites
    - Concept Explanation
    - Step-by-Step Technical Breakdown
    - Real-World Analogy
    - Hands-On Task
    - Code Example (Python + ROS 2)
    - "Common Mistakes & Debugging Tips" section
    - Mini-Assessment (4-6 questions)
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T030 [US2] Create and import a URDF model into Gazebo or Unity as described in content for Module 2.
- [ ] T031 [US2] Implement simulated sensors and robot joint control as described in content for Module 2.
- [ ] T032 [US2] Develop and integrate code examples (Python + ROS 2) for Module 2.
- [ ] T033 [US2] Verify that all code examples for Module 2 run successfully and demonstrate the learning objectives.
- [ ] T034 [US2] Create a test suite to validate the independent test criteria for Module 2: `docs/module-02-simulation/tests/test_simulation_control.py`.

---

## Phase 4: User Story 3 - Module 3: The AI-Robot Brain (NVIDIA Isaac) (P3)

Each module will contain four chapters, and each chapter will contain 3 to 4 lessons, as defined in `plan.md` and `spec.md`.

**Story Goal**: As a learner, I want to integrate advanced AI perception and navigation capabilities into my robot so that I can enable autonomous operation in a simulated environment.
**Independent Test Criteria**: The learner can run Isaac ROS VSLAM to build a map and use Nav2 to have the humanoid navigate to a goal.

- [ ] T035 [US3] Create `module-03-nvidia-isaac.md` as the main module overview file for Module 3 in `docs/module-03-nvidia-isaac/module-03-nvidia-isaac.md` with appropriate Docusaurus frontmatter.
- [ ] T036 [US3] For each chapter in Module 3, create a chapter overview Markdown file (e.g., `chapter-01-isaac-sim.md` in `docs/module-03-nvidia-isaac/chapter-01-isaac-sim/chapter-01-isaac-sim.md`) including:
    - Chapter Title
    - Chapter Summary
    - "Why this chapter matters" section
    - "Real Robotics Use-Cases" section
    - "Skills Students Will Build" section
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T037 [US3] For each lesson in Module 3, create a lesson Markdown file (e.g., `lesson-01-installation.md` in `docs/module-03-nvidia-isaac/chapter-01-isaac-sim/lesson-01-installation.md`) including:
    - Lesson Title & Objective
    - Prerequisites
    - Concept Explanation
    - Step-by-Step Technical Breakdown
    - Real-World Analogy
    - Hands-On Task
    - Code Example (Python + ROS 2)
    - "Common Mistakes & Debugging Tips" section
    - Mini-Assessment (4-6 questions)
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T038 [US3] Implement Isaac ROS VSLAM to build a map as described in content for Module 3.
- [ ] T039 [US3] Integrate Nav2 for humanoid navigation to a goal as described in content for Module 3.
- [ ] T040 [US3] Develop and integrate code examples (Python + ROS 2) for Module 3.
- [ ] T041 [US3] Verify that all code examples for Module 3 run successfully and demonstrate the learning objectives.
- [ ] T042 [US3] Create a test suite to validate the independent test criteria for Module 3: `docs/module-03-nvidia-isaac/tests/test_isaac_nav2.py`.

---

## Phase 5: User Story 4 - Module 4: Vision-Language-Action Robotics (VLA) (P4)

Each module will contain four chapters, and each chapter will contain 3 to 4 lessons, as defined in `plan.md` and `spec.md`.

**Story Goal**: As a learner, I want to build a system that connects a large language model to a robot's perception and action capabilities so that I can command the robot using natural language.
**Independent Test Criteria**: The learner can give a voice command to the robot, have an LLM process it, and see the robot execute the corresponding ROS 2 action.

- [ ] T043 [US4] Create `module-04-vla.md` as the main module overview file for Module 4 in `docs/module-04-vla/module-04-vla.md` with appropriate Docusaurus frontmatter.
- [ ] T044 [US4] For each chapter in Module 4, create a chapter overview Markdown file (e.g., `chapter-01-voice.md` in `docs/module-04-vla/chapter-01-voice/chapter-01-voice.md`) including:
    - Chapter Title
    - Chapter Summary
    - "Why this chapter matters" section
    - "Real Robotics Use-Cases" section
    - "Skills Students Will Build" section
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T045 [US4] For each lesson in Module 4, create a lesson Markdown file (e.g., `lesson-01-whisper-integration.md` in `docs/module-04-vla/chapter-01-voice/lesson-01-whisper-integration.md`) including:
    - Lesson Title & Objective
    - Prerequisites
    - Concept Explanation
    - Step-by-Step Technical Breakdown
    - Real-World Analogy
    - Hands-On Task
    - Code Example (Python + ROS 2)
    - "Common Mistakes & Debugging Tips" section
    - Mini-Assessment (4-6 questions)
    - Appropriate Docusaurus frontmatter (id, title, sidebar_position, description).
- [ ] T046 [US4] Implement voice command processing with an LLM as described in content for Module 4.
- [ ] T047 [US4] Integrate LLM output to execute corresponding ROS 2 actions as described in content for Module 4.
- [ ] T048 [US4] Develop and integrate code examples (Python + ROS 2) for Module 4.
- [ ] T049 [US4] Verify that all code examples for Module 4 run successfully and demonstrate the learning objectives.
- [ ] T050 [US4] Create a test suite to validate the independent test criteria for Module 4: `docs/module-04-vla/tests/test_vla_system.py`.

---

## Phase 6: Foundational Tasks (RAG Backend)

- [ ] T013 Create the main backend directory: `mkdir backend`
- [ ] T014 Create Python project structure within `backend/` as defined in `plan.md`:
    - `backend/app/api/__init__.py`
    - `backend/app/api/endpoints.py` (empty for now)
    - `backend/app/core/__init__.py`
    - `backend/app/core/config.py` (empty for now)
    - `backend/app/core/security.py` (empty for now)
    - `backend/app/services/__init__.py`
    - `backend/app/services/chunking.py` (empty for now)
    - `backend/app/services/embedding.py` (empty for now)
    - `backend/app/services/qdrant_service.py` (empty for now)
    - `backend/app/services/postgres_service.py` (empty for now)
    - `backend/app/models/__init__.py`
    - `backend/app/models/api_models.py` (empty for now)
    - `backend/app/models/db_models.py` (empty for now)
    - `backend/app/__init__.py`
    - `backend/app/main.py` (empty for now)
    - `backend/.env` (empty for now)
    - `backend/requirements.txt` (empty for now)
- [ ] T015 Define API models in `backend/app/models/api_models.py` based on `plan.md` and `openapi.json` (`EmbedRequest`, `EmbedResponse`, `SearchRequest`, `SearchResult`, `SearchResponse`, `ChatRequest`, `ChatResponse`).
- [ ] T016 Define database models/entities for `users`, `chats`, `message_logs`, and `message_citations` in `backend/app/models/db_models.py` based on `data-model.md`.
- [ ] T017 Implement SQL schema for Neon Postgres in `backend/sql_schema.sql` based on `plan.md` and `data-model.md` (`users`, `chats`, `message_logs`, `message_citations`).
- [ ] T018 Configure Qdrant vector store collection (`physical_ai_book_v1`) with specified vector size (1536), distance metric (Cosine), and payload schema (`source`, `chapter`, `module`, `content_hash`) in `backend/app/services/qdrant_service.py`.
- [ ] T019 Define text chunking strategy in `backend/app/services/chunking.py` (Recursive Character Text Splitting, primary separators, chunk size 1000, overlap 200, metadata tracking).

---

## Final Phase: Polish & Cross-Cutting Concerns

- [ ] T051 [P] Embed the chatbot UI component into the Docusaurus frontend.
- [ ] T052 [P] Create an API proxy layer in Docusaurus to securely communicate with the FastAPI backend.
- [ ] T053 Implement "selected text only" question flow in the Docusaurus chatbot UI.
- [ ] T054 Implement loading and error states UI for the chatbot in Docusaurus.
- [ ] T055 [P] Implement `GET /sessions/{sessionId}/messages` endpoint in `backend/app/api/endpoints.py` to retrieve chat history.
- [ ] T056 [P] Implement `POST /sessions/{sessionId}/messages` endpoint in `backend/app/api/endpoints.py` for new messages, including RAG logic (logging, search, LLM call, response, source logging).
- [ ] T057 [P] Implement `POST /embed` endpoint in `backend/app/api/endpoints.py` for content embedding.
- [ ] T058 [P] Implement `POST /search` endpoint in `backend/app/api/endpoints.py` for semantic search.
- [ ] T059 Test embedding process end-to-end (Docusaurus content -> FastAPI -> Qdrant).
- [ ] T060 Prepare local demo environment for Docusaurus and FastAPI.
- [ ] T061 Create deployment scripts/configurations for cloud deployment (e.g., Docker Compose, GitHub Actions workflows).
- [ ] T062 Prepare fallback offline demo strategy.
- [ ] T063 Create presentation workflow and materials.

---

## Dependencies

- Phase 1 (Docusaurus Setup) must be completed before content creation tasks in User Story Phases (Phases 2-5).
- User Story Phases (2-5) can be tackled in priority order (P1, P2, P3, P4) or in parallel if resources allow and dependencies are managed.
- Phase 6 (Foundational RAG Backend) must be completed before implementing specific RAG features in the Final Phase.
- Final Phase tasks are generally cross-cutting and should be addressed after core functionality is in place.

## Parallel Execution Examples

- **Docusaurus Frontend & Module Content Creation**: Phase 1 and the content generation for Modules 1-4 (Phases 2-5) can proceed in parallel to a large extent.
- **Backend Development**: Phase 6 (Foundational RAG Backend) and the backend-related tasks in the Final Phase can be worked on in parallel with the frontend and content development.

## Implementation Strategy

The project will follow an MVP (Minimum Viable Product) approach, with incremental delivery. The primary MVP will focus on completing User Story 1 (Module 1) and establishing the basic Docusaurus site and a functional, albeit minimal, RAG backend. Subsequent modules will be integrated incrementally.
