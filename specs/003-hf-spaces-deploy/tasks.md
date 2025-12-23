# Tasks: Hugging Face Spaces Backend Deployment

**Input**: Design documents from `/specs/003-hf-spaces-deploy/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, quickstart.md

**Tests**: No tests required for this deployment feature (deployment files, not application code).

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3, US4)
- Include exact file paths in descriptions

## User Stories Summary

| Story | Title | Priority |
|-------|-------|----------|
| US1 | Deploy Backend to Hugging Face Spaces | P1 |
| US2 | Configure Environment Secrets | P1 |
| US3 | Connect Frontend to HF Spaces Backend | P2 |
| US4 | Provide Deployment Documentation | P2 |

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create the HF Spaces deployment directory structure

- [X] T001 Create `huggingface-spaces/` directory at repository root
- [X] T002 [P] Create `huggingface-spaces/app/` directory structure mirroring `backend/app/`
- [X] T003 [P] Create empty `__init__.py` files in all `huggingface-spaces/app/` subdirectories

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Copy existing backend code that all deployment artifacts depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 [P] Copy `backend/app/main.py` to `huggingface-spaces/app/main.py`
- [X] T005 [P] Copy `backend/app/api/chat.py` to `huggingface-spaces/app/api/chat.py`
- [X] T006 [P] Copy `backend/app/api/health.py` to `huggingface-spaces/app/api/health.py`
- [X] T007 [P] Copy `backend/app/core/config.py` to `huggingface-spaces/app/core/config.py`
- [X] T008 [P] Copy `backend/app/db/postgres.py` to `huggingface-spaces/app/db/postgres.py`
- [X] T009 [P] Copy `backend/app/db/qdrant.py` to `huggingface-spaces/app/db/qdrant.py`
- [X] T010 [P] Copy `backend/app/models/schemas.py` to `huggingface-spaces/app/models/schemas.py`
- [X] T011 [P] Copy `backend/app/services/embedding.py` to `huggingface-spaces/app/services/embedding.py`
- [X] T012 [P] Copy `backend/app/services/rag_service.py` to `huggingface-spaces/app/services/rag_service.py`
- [X] T013 Copy `backend/requirements.txt` to `huggingface-spaces/requirements.txt`

**Checkpoint**: All application code is in place - deployment file creation can now begin

---

## Phase 3: User Story 1 - Deploy Backend to HF Spaces (Priority: P1) 🎯 MVP

**Goal**: Create HF Spaces-compatible Dockerfile and README.md for successful deployment

**Independent Test**: Deploy to HF Spaces, access `/api/health` endpoint, verify HTTP 200 response

### Implementation for User Story 1

- [X] T014 [US1] Create HF Spaces Dockerfile at `huggingface-spaces/Dockerfile` with:
  - Base image: `python:3.11-slim`
  - System deps: `gcc`, `g++`, `curl`
  - Non-root user: `useradd -m -u 1000 user`
  - `USER user` before file operations
  - `--chown=user` on all COPY commands
  - HEALTHCHECK instruction with curl
  - EXPOSE 7860
  - CMD: uvicorn on port 7860

- [X] T015 [US1] Create HF Spaces README.md at `huggingface-spaces/README.md` with YAML frontmatter:
  - `title: RAG Chatbot Backend`
  - `emoji: 🤖`
  - `colorFrom: blue`
  - `colorTo: purple`
  - `sdk: docker`
  - `app_port: 7860`
  - `short_description: FastAPI RAG chatbot with Gemini + Qdrant`
  - Add Space description markdown after frontmatter

- [X] T016 [US1] Validate Dockerfile builds locally with `docker build -t hf-rag-test huggingface-spaces/`

**Checkpoint**: User Story 1 complete - Dockerfile and README.md ready for HF Spaces upload

---

## Phase 4: User Story 2 - Configure Environment Secrets (Priority: P1)

**Goal**: Document all required secrets and their configuration in HF Space Settings

**Independent Test**: Set secrets in HF Settings, restart Space, verify health endpoint shows all services connected

### Implementation for User Story 2

- [X] T017 [US2] Create secrets reference table in `huggingface-spaces/README.md` listing:
  - `GEMINI_API_KEY` - Google Gemini API key
  - `QDRANT_URL` - Qdrant Cloud cluster URL
  - `QDRANT_API_KEY` - Qdrant Cloud API key
  - `QDRANT_COLLECTION` - Collection name (docusaurus-book)
  - `DATABASE_URL` - Neon PostgreSQL connection string with sslmode=require
  - `ALLOWED_ORIGINS` - Comma-separated frontend URLs
  - `LOG_LEVEL` - Logging verbosity (INFO)

- [X] T018 [US2] Add "Configuration" section to `huggingface-spaces/README.md` with step-by-step secret setup instructions

- [X] T019 [US2] Document environment variable validation behavior (what happens when secrets are missing)

**Checkpoint**: User Story 2 complete - Secret documentation ready for developer reference

---

## Phase 5: User Story 3 - Connect Frontend to HF Spaces Backend (Priority: P2)

**Goal**: Document frontend configuration to connect Vercel app to HF Spaces backend

**Independent Test**: Update Vercel env var, redeploy frontend, verify chat works end-to-end

### Implementation for User Story 3

- [X] T020 [US3] Add "Frontend Configuration" section to `specs/003-hf-spaces-deploy/quickstart.md` documenting:
  - Environment variable: `REACT_APP_API_BASE_URL`
  - Format: `https://USERNAME-rag-chatbot.hf.space`
  - Vercel dashboard navigation steps
  - Redeploy requirement after env var change

- [X] T021 [US3] Document CORS configuration requirements in `huggingface-spaces/README.md`:
  - `ALLOWED_ORIGINS` must include Vercel frontend URL
  - Multiple origins: comma-separated format
  - Include HF Spaces URL for Swagger UI testing

- [X] T022 [US3] Add verification steps for frontend connectivity in `specs/003-hf-spaces-deploy/quickstart.md`:
  - Check browser console for CORS errors
  - Verify chat messages stream correctly
  - Test SSE streaming functionality

**Checkpoint**: User Story 3 complete - Frontend can connect to deployed backend

---

## Phase 6: User Story 4 - Provide Deployment Documentation (Priority: P2)

**Goal**: Create comprehensive step-by-step deployment guide

**Independent Test**: Follow documentation to deploy backend without additional assistance

### Implementation for User Story 4

- [X] T023 [US4] Verify `specs/003-hf-spaces-deploy/quickstart.md` covers:
  - Step 1: Create HF Space (with screenshots)
  - Step 2: Configure Secrets (complete list)
  - Step 3: Upload Deployment Files (Git and Web options)
  - Step 4: Monitor Build (expected log output)
  - Step 5: Verify Deployment (curl commands)
  - Step 6: Update Frontend (Vercel steps)

- [X] T024 [US4] Add troubleshooting section to `specs/003-hf-spaces-deploy/quickstart.md`:
  - Build failure causes and fixes
  - Health endpoint 503 causes
  - CORS error resolution
  - Cold start behavior explanation

- [X] T025 [US4] Add verification checklist to `specs/003-hf-spaces-deploy/quickstart.md`:
  - HF Space "Running" status
  - Health endpoint HTTP 200
  - All services connected
  - Frontend chat working
  - SSE streaming working

- [X] T026 [US4] Document redeployment process for code updates in `specs/003-hf-spaces-deploy/quickstart.md`

**Checkpoint**: User Story 4 complete - Documentation enables first-attempt deployment success

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and cleanup

- [X] T027 [P] Validate all file paths in `huggingface-spaces/` match expected structure
- [X] T028 [P] Verify `requirements.txt` includes all dependencies from `backend/requirements.txt`
- [X] T029 [P] Test Docker build locally one final time
- [X] T030 Validate `specs/003-hf-spaces-deploy/quickstart.md` is complete and unambiguous
- [X] T031 Remove any `.pyc` or `__pycache__` files from `huggingface-spaces/`

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3-6)**: All depend on Foundational phase completion
  - US1 and US2 can proceed in parallel (both P1)
  - US3 and US4 can proceed in parallel (both P2)
- **Polish (Phase 7)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational - Creates Dockerfile and README.md
- **User Story 2 (P1)**: Can start after Foundational - Documents secrets in README.md
- **User Story 3 (P2)**: Can start after Foundational - Documents frontend connection
- **User Story 4 (P2)**: Depends on US1, US2, US3 content being available for comprehensive documentation

### Parallel Opportunities

**Within Phase 2 (Foundational):**
All file copy tasks (T004-T012) can run in parallel - different files, no dependencies

**Within Phase 3 (US1):**
T014 and T015 can run in parallel - Dockerfile and README.md are separate files

**Across User Stories:**
- US1 (Dockerfile/README.md creation) and US2 (secrets documentation) can run in parallel
- US3 (frontend docs) and US4 (deployment docs) can run in parallel

---

## Parallel Example: Phase 2 (Foundational)

```bash
# Launch all file copy tasks together:
Task: "Copy backend/app/main.py to huggingface-spaces/app/main.py"
Task: "Copy backend/app/api/chat.py to huggingface-spaces/app/api/chat.py"
Task: "Copy backend/app/api/health.py to huggingface-spaces/app/api/health.py"
Task: "Copy backend/app/core/config.py to huggingface-spaces/app/core/config.py"
Task: "Copy backend/app/db/postgres.py to huggingface-spaces/app/db/postgres.py"
Task: "Copy backend/app/db/qdrant.py to huggingface-spaces/app/db/qdrant.py"
Task: "Copy backend/app/models/schemas.py to huggingface-spaces/app/models/schemas.py"
Task: "Copy backend/app/services/embedding.py to huggingface-spaces/app/services/embedding.py"
Task: "Copy backend/app/services/rag_service.py to huggingface-spaces/app/services/rag_service.py"
```

---

## Implementation Strategy

### MVP First (User Story 1 + 2)

1. Complete Phase 1: Setup (create directories)
2. Complete Phase 2: Foundational (copy app code)
3. Complete Phase 3: User Story 1 (Dockerfile + README.md)
4. Complete Phase 4: User Story 2 (secrets documentation)
5. **STOP and VALIDATE**: Deploy to HF Spaces, verify health endpoint
6. Deploy/demo if ready - backend is functional!

### Incremental Delivery

1. Complete Setup + Foundational + US1 + US2 → Backend deployable (MVP!)
2. Add User Story 3 → Frontend can connect
3. Add User Story 4 → Documentation complete for handoff
4. Polish → Final validation

### Single Developer Strategy

Recommended order for solo implementation:

1. Phase 1: Setup (5 min)
2. Phase 2: Foundational - all file copies (10 min)
3. Phase 3: US1 - Dockerfile + README.md (15 min)
4. Phase 4: US2 - Secrets docs (10 min)
5. **Deploy and test** (15 min)
6. Phase 5: US3 - Frontend docs (10 min)
7. Phase 6: US4 - Complete quickstart (10 min)
8. Phase 7: Polish (5 min)

**Total estimated time**: 60-80 minutes

---

## Notes

- All tasks create/copy files to `huggingface-spaces/` directory (not modifying `backend/`)
- No test tasks included (deployment files, not application code)
- Dockerfile MUST use port 7860 and non-root user UID 1000
- README.md MUST have exact YAML frontmatter format for HF Spaces
- quickstart.md already exists - tasks verify and enhance it
