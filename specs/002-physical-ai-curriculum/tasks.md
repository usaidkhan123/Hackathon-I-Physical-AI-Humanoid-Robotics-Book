# Tasks: RAG Chatbot Implementation

**Input**: Design documents from `specs/002-physical-ai-curriculum/`
**Prerequisites**: `plan.md` and `spec.md` are required.

---

## Phase 1: Backend Setup (FastAPI)

**Purpose**: Initialize the backend server and create placeholder endpoints.

- [X] T001: Create the `backend/` directory.
- [X] T002: Create a `main.py` inside `backend/` with a basic FastAPI app.
- [X] T003: Implement a `/` root endpoint for health checks.
- [X] T004: Implement `POST /sessions/{sessionId}/messages` endpoint (stubbed response).
- [X] T005: Implement `GET /sessions/{sessionId}/messages` endpoint (stubbed response).
- [X] T006: Implement `POST /embed` endpoint (stubbed response).
- [X] T007: Implement `POST /search` endpoint (stubbed response).
- [X] T008: Configure CORS to allow requests from the Docusaurus frontend.
- [X] T009: Create a `requirements.txt` with `fastapi` and `uvicorn`.

---

## Phase 2: Frontend Implementation (Docusaurus)

**Purpose**: Create the chatbot UI component and integrate it.

- [X] T010: Create the `physical-ai-book/src/components/Chatbot/` directory.
- [X] T011: Create `Chatbot.tsx` with a basic React component structure.
- [X] T012: Add state management for user input, messages, loading, and error states.
- [X] T013: Implement the UI for displaying chat messages.
- [X] T014: Implement the UI for user input (text area and send button).
- [X] T015: Create `styles.module.css` for the chatbot component and add basic styling (borders, padding).
- [X] T016: Integrate the `Chatbot` component into `physical-ai-book/src/pages/index.tsx`.

---

## Phase 3: Frontend-Backend Integration

**Purpose**: Connect the chatbot UI to the backend API.

- [X] T017: Implement the `fetch` call in `Chatbot.tsx` to the `POST /sessions/{sessionId}/messages` endpoint.
- [X] T018: Handle the response from the backend and display it in the chat window.
- [X] T019: Implement loading and error state handling for the API call.
- [X] T020: [P] Implement UI logic to send selected text to the chatbot.

---

## Phase 4: Testing and Validation

**Purpose**: Ensure the chatbot is working end-to-end.

- [ ] T021: Manually test sending a message from the UI and receiving a response.
- [ ] T022: Verify that loading and error states are displayed correctly.
- [ ] T023: Verify that the chat history is maintained (if implemented).

---

## Execution Strategy

The phases are designed to be completed sequentially.

1.  **Phase 1 (Backend)**: Set up the server foundation.
2.  **Phase 2 (Frontend)**: Build the user interface.
3.  **Phase 3 (Integration)**: Connect the frontend and backend.
4.  **Phase 4 (Testing)**: Verify the end-to-end functionality.
