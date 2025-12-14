---
id: 002
title: Create Development Plan for Robotics Curriculum
stage: plan
date: 2025-12-07
surface: agent
model: gemini-pro
feature: 001-robotics-curriculum-spec
branch: 001-robotics-curriculum-spec
user: User
command: /sp.plan
labels: [plan, architecture, robotics]
links:
  spec: specs/001-robotics-curriculum-spec/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-robotics-curriculum-spec/plan.md
  - specs/001-robotics-curriculum-spec/research.md
  - specs/001-robotics-curriculum-spec/data-model.md
  - specs/001-robotics-curriculum-spec/contracts/openapi.json
  - specs/001-robotics-curriculum-spec/quickstart.md
  - GEMINI.md
tests:
  - none
---

## Prompt
```
/sp.plan You are creating a complete and production-ready development plan for a technical book titled:

"Teaching Physical AI & Humanoid Robotics — Hands-On Course Book for Beginners and Intermediate Learners"

The documentation framework must be built using Docusaurus.
The RAG chatbot must use OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud (free tier).
Gemini CLI + SpecKit will be used for project governance, planning, and prompt management.

The goal is to define EVERYTHING required to build this book.

---

## 1. Project Vision

Define the objectives of this book:
- Teach Physical AI (Embodied AI)
- Teach humanoid robotics foundations
- Provide hands-on labs and exercises
- Integrate an intelligent RAG chatbot inside the book
- Support beginner and intermediate learners

---

## 2. Full System Setup Plan

Provide an exact setup workflow including:

### 2.1 Core Tool Installation
- Node.js
- Docusaurus CLI
- Git
- Python
- FastAPI
- Qdrant Cloud setup
- Neon Postgres setup
- Gemini CLI
- SpecKit initialization

### 2.2 Docusaurus Initialization
Provide commands and configurations:
- Project creation
- Sidebar config
- Navbar config
- Versioning config

---

## 3. Repository & Folder Structure

Define required file structure:

/physical-ai-book
/docs
/module-01-foundations
/module-02-ros2
/module-03-simulation
/module-04-humanoids
/module-05-physical-ai
/module-06-rag-chatbot
/rag-backend
/database
/scripts
/prompts
/assets

yaml
Copy code

---

## 4. Content Production Workflow

Provide a strict content creation lifecycle:

- Constitution enforcement
- Specification drafting
- Plan validation
- Chapter drafting
- Lesson authoring
- Review and refinement

(No use of `sp.expand` or `sp.implement` commands.)

---

## 5. Chapter and Lesson Design Standard

Define a standard Markdown template for each lesson:

Frontmatter + Sections:
- Title
- Objectives
- Theory
- Practical Lab
- Code
- Common Errors
- Exercises
- Quiz

---

## 6. RAG Chatbot Architecture Plan

Define:

### 6.1 Chunking & Embedding
- Chunk size strategy
- Metadata tagging

### 6.2 Qdrant Strategy
- Collection structure

### 6.3 Neon Postgres Schema
- Tables for users, chats, citations

### 6.4 Agent Roles
- Retriever agent
- Answer agent
- Safety agent

---

## 7. Docusaurus + Chatbot Integration

Define:
- Frontend chat widget
- API routing
- Handling “selected text only” questions

---

## 8. Build, Test, and Deploy Pipeline

Include:
- Git workflow
- Preview build
- Production build
- Hosting strategy

---

## 9. Maintenance Strategy

Define:
- Content updates
- Vector re-indexing
- Database backup

---

## 10. Output Format

The generated plan must be:

✅ Extremely detailed  
✅ Step-by-step  
✅ Production ready  
✅ Beginner friendly

Finish with a project readiness checklist.
```

## Response snapshot

I have successfully generated the development plan and all associated design artifacts for the "Physical AI & Humanoid Robotics Curriculum".

1.  **Plan Created**: The main plan is at `specs/001-robotics-curriculum-spec/plan.md`.
2.  **Design Artifacts**: The following documents have been created:
    - `specs/001-robotics-curriculum-spec/research.md`
    - `specs/001-robotics-curriculum-spec/data-model.md`
    - `specs/001-robotics-curriculum-spec/contracts/openapi.json`
    - `specs/001-robotics-curriculum-spec/quickstart.md`
3.  **Agent Context Updated**: The agent's context was updated with the project's technology stack.

The project is now ready for the task breakdown phase (`/sp.tasks`).

## Outcome

- ✅ Impact: High. Created the architectural and design blueprint for the project.
- 🧪 Tests: N/A
- 📁 Files: 6 files created/updated.
- 🔁 Next prompts: The user can now proceed to the task breakdown phase with `/sp.tasks`.
- 🧠 Reflection: The process was smooth. The user's detailed prompt for the plan allowed for the creation of all necessary design artifacts without needing clarification. The `setup-plan.ps1` script worked as expected.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): PASS
- Prompt variant (if applicable): null
- Next experiment: N/A
