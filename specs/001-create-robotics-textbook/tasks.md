# Tasks: AI-Native Textbook for Physical AI & Humanoid Robotics

**Input**: Design documents from `specs/001-create-robotics-textbook/`
**Prerequisites**: plan.md, spec.md

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [X] T001 Create project structure per implementation plan
- [X] T002 Initialize Docusaurus project with Spec-Kit Plus template
- [X] T003 [P] Install all required frontend dependencies
- [X] T004 [P] Set up Python backend environment and install dependencies
- [X] T005 [P] Configure environment variables for both frontend and backend

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

- [X] T006 Set up Neon PostgreSQL database and create initial schemas (Manual step: User needs to set up Neon DB and create schemas)
- [X] T007 Set up Qdrant Cloud and configure collections (Manual step: User needs to set up Qdrant Cloud and configure collections)
- [X] T008 Implement basic API endpoints in FastAPI
- [X] T009 Create base data models in the backend

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Core Textbook Experience (Priority: P1) 🎯 MVP

**Goal**: As a student, I want to access a comprehensive online textbook on "Physical AI & Humanoid Robotics" so that I can learn about the subject in an interactive and structured manner.

**Independent Test**: The Docusaurus site is deployed and accessible, and the content for at least one module is present and readable.

### Implementation for User Story 1

- [X] T010 [US1] Create custom landing page with hero section and module cards in `frontend/src/pages/index.tsx`
- [X] T011 [P] [US1] Build module cards component in `frontend/src/components/ModuleCards.tsx`
- [X] T012 [P] [US1] Create hardware comparison table component in `frontend/src/components/HardwareTable.tsx`
- [X] T013 [P] [US1] Implement cost calculator component in `frontend/src/components/CostCalculator.tsx`
- [X] T014 [US1] Generate directory structure for Module 1 in `frontend/docs/module-1`
- [X] T015 [US1] Generate "ROS 2 Fundamentals" chapter content in `frontend/docs/module-1/ros-fundamentals.md`
- [X] T016 [US1] Generate "URDF Modeling" chapter content in `frontend/docs/module-1/urdf-modeling.md`
- [X] T017 [US1] Create "ROS 2 Nodes & Topics Chapter" content in `frontend/docs/module-1/ros-nodes-topics.md`

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Interactive Learning with RAG Chatbot (Priority: P2)

**Goal**: As a student, I want to use an intelligent chatbot that can answer my questions about the textbook content, so that I can get instant clarification and deeper insights.

**Independent Test**: The chatbot is available on the website and can answer a question about a specific chapter.

### Implementation for User Story 2

- [X] T017 [US2] Set up vector embedding pipeline for content ingestion in `backend/src/services/ingestion.py`
- [X] T018 [US2] Implement `/chat` and `/selected-chat` endpoints in `backend/src/api/chat.py`
- [X] T019 [US2] Integrate Gemini API for response generation in `backend/src/services/chatbot.py`
- [X] T020 [P] [US2] Build chatbot component in `frontend/src/components/Chatbot.tsx`
- [X] T021 [US2] Implement selected text analysis functionality in the frontend

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Personalized Learning Path (Priority: P3)

**Goal**: As a user, I want to create an account and provide my background information, so that the textbook content can be personalized to my learning style and experience level.

**Independent Test**: A user can sign up, log in, and see a personalized version of a chapter.

### Implementation for User Story 3

- [X] T022 [US3] Set up Better Auth backend for user authentication in `backend/src/api/auth.py`
- [X] T023 [P] [US3] Create signup form with background questions in `frontend/src/pages/signup.tsx`
- [X] T024 [P] [US3] Build user dashboard in `frontend/src/pages/dashboard.tsx`
- [X] T025 [US3] Implement content adaptation logic in `backend/src/services/personalization.py`
- [X] T026 [P] [US3] Create personalization button component in `frontend/src/components/PersonalizationButton.tsx`

---

## Phase 6: User Story 4 - Multilingual Support (Priority: P4)

**Goal**: As a student who is more comfortable in Urdu, I want to be able to translate the textbook content into Urdu, so that I can understand the material better.

**Independent Test**: A user can click a button to see the Urdu translation of a chapter.

### Implementation for User Story 4

- [X] T027 [US4] Integrate Gemini Translation API in `backend/src/services/translation.py`
- [X] T028 [P] [US4] Create translation button component in `frontend/src/components/TranslationButton.tsx`
- [X] T029 [P] [US4] Implement bilingual display component in `frontend/src/components/BilingualDisplay.tsx`

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [X] T030 [P] Documentation updates in `frontend/docs/`
- [X] T031 Code cleanup and refactoring across the project
- [X] T032 Performance optimization for all pages and components
- [X] T033 Security hardening for backend and frontend
- [X] T034 Run quickstart.md validation

---

## Phase 8: Full Content Generation and Docusaurus Build

**Purpose**: Generate all remaining book content and build the Docusaurus site.

- [X] T035 Generate directory structure for Module 2 in `frontend/docs/module-2`
- [X] T036 Create "Gazebo Simulation Chapter" content in `frontend/docs/module-2/gazebo-simulation.md`
- [X] T037 Create "Sensor Simulation Chapter" content in `frontend/docs/module-2/sensor-simulation.md`
- [X] T038 Generate directory structure for Module 3 in `frontend/docs/module-3`
- [X] T039 Create "Isaac Sim Setup Guide" chapter content in `frontend/docs/module-3/isaac-sim-setup.md`
- [X] T040 Create "Perception Pipeline" chapter content in `frontend/docs/module-3/perception-pipeline.md`
- [X] T041 Generate directory structure for Module 4 in `frontend/docs/module-4`
- [X] T042 Create "OpenAI Whisper Integration" chapter content in `frontend/docs/module-4/whisper-integration.md`
- [X] T043 Create "LLM Task Planning" chapter content in `frontend/docs/module-4/llm-task-planning.md`
- [X] T044 Build Docusaurus site


### Phase Dependencies

- **Setup (Phase 1)**: No dependencies
- **Foundational (Phase 2)**: Depends on Setup
- **User Stories (Phase 3-6)**: Depend on Foundational
- **Polish (Phase 7)**: Depends on all user stories

### User Story Dependencies

- **User Story 1**: Depends on Foundational
- **User Story 2**: Depends on Foundational
- **User Story 3**: Depends on Foundational
- **User Story 4**: Depends on Foundational

### Parallel Opportunities

- All tasks marked with [P] can be worked on in parallel.
- All user stories can be worked on in parallel after the Foundational phase is complete.

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
