---
description: "Task list template for creating a book chapter"
---

# Tasks: Chapter [CHAPTER_NUMBER]: [CHAPTER_TITLE]

**Input**: Design documents from `/specs/chapters/[###-chapter-name]/`
**Prerequisites**: `plan.md` and `spec.md` are required.

**Organization**: Tasks are grouped into phases that follow the Spec-Driven Development workflow defined in the Constitution (Section VII).

---

## Phase 1: Planning & Specification (`/sp.plan`)

**Purpose**: Define the chapter's scope, learning objectives, and structure.

- [ ] T001: Generate the initial `spec.md` defining learning objectives and outcomes.
- [ ] T002: Generate the initial `plan.md` outlining the technical context and passing the Constitution Check.
- [ ] T003: Finalize the chapter's detailed outline within `plan.md`.

---

## Phase 2: Content Expansion & Initial Draft (`/sp.expand`)

**Purpose**: Write the primary narrative and explanatory content of the chapter.

- [ ] T004: Draft the introductory section, setting the stage for the chapter.
- [ ] T005: Write the core theoretical explanations for [Concept 1]. Use analogies as required by Pedagogy Law.
- [ ] T006: Write the core theoretical explanations for [Concept 2]. Use analogies as required by Pedagogy Law.
- [ ] T007: Draft the hands-on demonstration sections that precede the theory.
- [ ] T008: Create placeholder sections for all items required by the Chapter Completeness Checklist (diagrams, code, quiz, etc.).

---

## Phase 3: Implementation of Assets (`/sp.implement`)

**Purpose**: Generate all non-prose assets required for the chapter.

- [ ] T009: Generate the system diagram for [System Name] and place it in `diagrams/`.
- [ ] T010: Write the complete, working code for the primary code example and place it in `code/`.
- [ ] T011: Add detailed comments to the code example.
- [ ] T012: Develop the code and instructions for the robotics mini-project.
- [ ] T013: Create the "Hands-On Assignment" description and any starter code required.

---

## Phase 4: Chapter Completion & Review

**Purpose**: Finalize the chapter and ensure it meets all constitutional requirements.

- [ ] T014: Write the 5-10 questions for the "Test Your Understanding" quiz.
- [ ] T015: Add explicit safety notes related to any hardware interaction.
- [ ] T016: Generate the chapter summary using `/sp.summarize`.
- [ ] T017: Review the entire chapter against all checklists in `spec.md` and `plan.md`.
- [ ] T018: Proofread the entire chapter for clarity, grammar, and style.

---

## Execution Strategy

The phases are designed to be completed sequentially.

1.  **Phase 1 (Planning)** must be completed and approved before any content is written.
2.  **Phase 2 (Drafting)** builds the narrative backbone of the chapter.
3.  **Phase 3 (Assets)** provides the critical hands-on materials. It can be done in parallel with Phase 2 if desired.
4.  **Phase 4 (Completion)** is the final quality gate before the chapter is considered "done."

This structured approach ensures every chapter is compliant with the constitution and complete before being merged.