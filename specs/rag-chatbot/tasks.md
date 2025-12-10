# Tasks: RAG-Powered Chatbot for Docusaurus

**Input**: Design documents from `/specs/rag-chatbot/`
**Prerequisites**: plan.md (complete), spec.md (6 user stories), ADRs (011-015)

**Tests**: Test tasks included per FR-018, FR-020 requirements (pytest backend, evaluation script)

**Organization**: Tasks grouped by user story to enable independent implementation and testing. Note: US3 (Ingestion) is foundational and blocks US1/US6.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4, US5, US6)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `backend/app/`, `backend/scripts/`, `backend/tests/`
- **Frontend**: `src/components/`, `src/hooks/`, `src/api/`, `src/theme/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and environment setup

- [ ] T001 Create backend directory structure: backend/app/{api,services,models,db,core}
- [ ] T002 Create backend subdirectories: backend/{scripts,tests/{unit,integration,fixtures}}
- [ ] T003 Create all __init__.py files per plan.md:66-78
- [ ] T004 Create backend/requirements.txt with exact versions from plan.md:82-97
- [ ] T005 Create backend/.env.example with Gemini/Qdrant/Postgres/CORS config from plan.md:101-120
- [ ] T006 [P] Create frontend directories: src/{components/ChatWidget,hooks,api,theme}
- [ ] T007 [P] Install frontend dependencies: react-markdown, remark-gfm per plan.md:883

**Checkpoint**: Project structure matches plan.md, dependencies ready for installation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core configuration and database clients - MUST complete before user stories

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T008 Create backend/app/core/config.py with Pydantic Settings per plan.md:136-166
- [ ] T009 Create backend/app/db/qdrant.py with QdrantManager class per plan.md:182-244
- [ ] T010 [P] Create backend/app/services/embedding.py with Gemini text-embedding-004 integration per plan.md:262-298
- [ ] T011 Test embedding service: verify 768-dim output per plan.md:303-311
- [ ] T012 Test Qdrant connection: verify collection creation per plan.md:249-256

**Checkpoint**: Foundation ready - config loads, Qdrant connects, embeddings generate

---

## Phase 3: User Story 3 - Document Ingestion (Priority: P1) 🎯 FOUNDATIONAL

**Goal**: Index all 21 Docusaurus markdown files (~150-200 chunks) into Qdrant with 768-dim Gemini embeddings

**Independent Test**: Run `python backend/scripts/ingest.py`, verify Qdrant collection contains ~150-200 vectors with metadata

**⚠️ BLOCKS**: US1 (Basic Chat) and US6 (Evaluation) depend on ingestion completing

### Implementation for User Story 3

- [X] T013 [P] [US3] Create backend/scripts/chunker.py with DocumentChunker class per plan.md:318-408
- [X] T014 [P] [US3] Add tiktoken token counting to chunker per plan.md:329-330
- [X] T015 [US3] Implement parse_markdown() with H2/H3 boundary detection per plan.md:332-373
- [X] T016 [US3] Implement _create_chunk() with heading hierarchy preservation per plan.md:375-392
- [X] T017 [US3] Implement _truncate_to_tokens() for 512 token limit per plan.md:394-408
- [X] T018 [US3] Create backend/scripts/ingest.py with async document ingestion per plan.md:416-479
- [X] T019 [US3] Add markdown file discovery (Path.rglob) per plan.md:446-447
- [X] T020 [US3] Add batch embedding generation per plan.md:463-465
- [X] T021 [US3] Add Qdrant upsert with UUID assignment per plan.md:468-474
- [X] T022 [US3] Add error handling for failed files per plan.md:457-458
- [X] T023 [US3] Run ingestion script, verify ~150-200 chunks indexed per plan.md:486-511

**Checkpoint**: Qdrant collection `docusaurus-book` contains all vectors, metadata includes heading_path

---

## Phase 4: User Story 1 - Basic Chat Interaction (Priority: P1) 🎯 MVP CORE

**Goal**: Enable readers to ask questions and get AI answers with source citations via chat widget

**Independent Test**: Open chat widget, ask "What is physical AI?", verify response with citations

**Dependencies**: Requires US3 (Ingestion) complete

### Backend Implementation for User Story 1

- [X] T024 [P] [US1] Create backend/app/models/schemas.py with ChatRequest/ChatResponse/Source models per plan.md:536-555
- [X] T025 [P] [US1] Create backend/app/services/llm.py with Gemini streaming via OpenAI SDK per plan.md:562-623
- [X] T026 [US1] Create backend/app/services/rag_service.py with query_general() per plan.md:630-683
- [X] T027 [US1] Implement RAG pipeline: embedding → search → LLM streaming per plan.md:645-671
- [X] T028 [US1] Create backend/app/api/chat.py with POST /chat endpoint per plan.md:690-735
- [X] T029 [US1] Implement SSE streaming response per plan.md:714-730
- [X] T030 [US1] Add source citation extraction per plan.md:656-662
- [X] T031 [P] [US1] Create backend/app/api/health.py with GET /health endpoint per plan.md:742-777
- [X] T032 [US1] Create backend/app/main.py with FastAPI app + CORS + routers per plan.md:783-817
- [X] T033 [US1] Test backend: curl /api/health, verify Qdrant/Gemini status per plan.md:823-838
- [X] T034 [US1] Test backend: curl /api/chat, verify SSE streaming per plan.md:840-852

### Frontend Implementation for User Story 1

- [X] T035 [P] [US1] Create src/api/chatClient.ts with SSE streaming consumer per plan.md:890-963
- [X] T036 [P] [US1] Create src/hooks/useChat.ts with message state management per plan.md:1014-1088
- [X] T037 [P] [US1] Create src/components/ChatWidget/ChatWidget.tsx with floating button + panel per plan.md:1096-1209
- [X] T038 [P] [US1] Create src/components/ChatWidget/ChatWidget.module.css with responsive design per plan.md:1217-1377
- [X] T039 [US1] Swizzle Docusaurus Root component per plan.md:1384-1404
- [X] T040 [US1] Test frontend: verify chat widget appears, sends message, streams response per plan.md:1410-1434

**Checkpoint**: Chat widget functional end-to-end, answers questions with source citations

---

## Phase 5: User Story 2 - Text Selection Mode (Priority: P2)

**Goal**: Enable focused Q&A based on highlighted text only (not full book context)

**Independent Test**: Highlight paragraph, ask "Explain this", verify answer uses ONLY selected text

**Dependencies**: None (independent of US1, can work standalone)

### Implementation for User Story 2

- [X] T041 [P] [US2] Create src/hooks/useTextSelection.ts with mouseup detection per plan.md:969-1006
- [X] T042 [P] [US2] Add 8000 char truncation to useTextSelection per plan.md:988-993
- [X] T043 [US2] Add selection mode to backend/app/services/rag_service.py query_selection() per plan.md:673-681
- [X] T044 [US2] Update ChatWidget.tsx to show "Selection Mode Active" badge per plan.md:1140-1145
- [X] T045 [US2] Update useChat.ts to pass mode + context to backend per plan.md:1117-1119
- [X] T046 [US2] Add clearSelection() button to badge per plan.md:1143
- [X] T047 [US2] Test selection mode: highlight text, verify badge, ask question, verify constrained answer per plan.md:1430-1431

**Checkpoint**: Selection mode works independently, constrains answers to highlighted text only

---

## Phase 6: User Story 6 - RAG Quality Evaluation (Priority: P1) 🎯 VALIDATION

**Goal**: Verify RAG retrieval accuracy ≥90% via golden dataset (SC-004, SC-012)

**Independent Test**: Run `python backend/scripts/evaluate.py`, verify ≥90% accuracy at >0.7 similarity

**Dependencies**: Requires US3 (Ingestion) complete

### Implementation for User Story 6

- [ ] T048 [P] [US6] Create backend/tests/fixtures/golden_dataset.json with 20 domain + 5 OOS questions per plan.md:1885-1913
- [ ] T049 [P] [US6] Create backend/tests/unit/test_embedding.py with embedding dimension tests per plan.md:1827-1845
- [ ] T050 [P] [US6] Create backend/tests/integration/test_chat_api.py with endpoint tests per plan.md:1850-1877
- [ ] T051 [US6] Create backend/scripts/evaluate.py with RAG accuracy evaluation per plan.md:1920-2007
- [ ] T052 [US6] Implement domain question evaluation (expected chunks, >0.7 sim) per plan.md:1945-1971
- [ ] T053 [US6] Implement OOS question evaluation (low similarity <0.5) per plan.md:1974-1988
- [ ] T054 [US6] Add accuracy metrics: precision@5, recall@5, pass/fail per plan.md:1991-2001
- [ ] T055 [US6] Run evaluation, verify ≥90% accuracy per plan.md:2014-2032
- [ ] T056 [US6] Run pytest unit tests, verify all pass per plan.md:2015-2016
- [ ] T057 [US6] Run pytest integration tests, verify all pass per plan.md:2018-2019
- [ ] T058 [US6] Check test coverage ≥80% backend per plan.md:2034-2036

**Checkpoint**: Evaluation passes (≥90%), all tests pass, coverage meets target

---

## Phase 7: User Story 4 - Conversation Persistence (Priority: P3)

**Goal**: Store conversation history in Postgres, allow resume across browser sessions

**Independent Test**: Have conversation, close browser, reopen, verify history retrievable

**Dependencies**: None (independent feature, optional for MVP)

### Implementation for User Story 4

- [ ] T059 [P] [US4] Create backend/app/db/postgres.py with PostgresManager class per plan.md:1447-1529
- [ ] T060 [P] [US4] Add CREATE TABLE statements for conversations + messages per plan.md:1468-1484
- [ ] T061 [P] [US4] Implement create_conversation() per plan.md:1486-1492
- [ ] T062 [P] [US4] Implement save_message() per plan.md:1494-1508
- [ ] T063 [P] [US4] Implement get_conversation_history() per plan.md:1510-1527
- [ ] T064 [US4] Update backend/app/api/chat.py to save messages during streaming per plan.md:1537-1601
- [ ] T065 [US4] Add GET /conversation/{id} endpoint per plan.md:1604-1608
- [ ] T066 [US4] Add startup/shutdown lifecycle hooks in main.py per plan.md:1618-1629
- [ ] T067 [US4] Test conversation persistence: send messages, query by conversation_id per plan.md:1635-1651

**Checkpoint**: Conversations persist to Postgres, history retrievable by ID

---

## Phase 8: User Story 5 - Health Monitoring (Priority: P3)

**Goal**: Expose /health endpoint with Qdrant/Postgres/Gemini status

**Independent Test**: Call GET /health, verify all service statuses returned

**Dependencies**: None (already partially implemented in US1, this completes it)

### Implementation for User Story 5

- [ ] T068 [US5] Enhance backend/app/api/health.py to check Postgres status per plan.md:96-98
- [ ] T069 [US5] Add detailed error reporting for disconnected services per plan.md:97-99
- [ ] T070 [US5] Test health endpoint with all services up per plan.md:100
- [ ] T071 [US5] Test health endpoint with Qdrant down (mock) per plan.md:99

**Checkpoint**: Health endpoint returns comprehensive service status JSON

---

## Phase 9: Deployment & Docker (Cross-Cutting)

**Purpose**: Production deployment configuration

- [ ] T072 [P] Create backend/Dockerfile per plan.md:1670-1687
- [ ] T073 [P] Create docker-compose.yml in project root per plan.md:1693-1737
- [ ] T074 [P] Create .env.docker with all API keys per plan.md:1743-1753
- [ ] T075 [P] Update src/api/chatClient.ts for production backend URL per plan.md:1760-1764
- [ ] T076 [P] Create vercel.json for Docusaurus deployment per plan.md:1770-1774
- [ ] T077 Test Docker: docker-compose build per plan.md:1781-1782
- [ ] T078 Test Docker: docker-compose up, verify services start per plan.md:1784-1792
- [ ] T079 Test production build: npm run build per plan.md:1797-1800

**Checkpoint**: Docker deployment works, production build succeeds

---

## Phase 10: Polish & Documentation

**Purpose**: Final cleanup and documentation

- [ ] T080 [P] Update README.md with setup instructions
- [ ] T081 [P] Document environment variables in .env.example
- [ ] T082 [P] Add API documentation comments to endpoints
- [ ] T083 [P] Create deployment checklist per plan.md:2127-2139
- [ ] T084 Run final /api/health check per plan.md:2132
- [ ] T085 Run final evaluation script per plan.md:2131
- [ ] T086 Test end-to-end: ingest → chat → verify citations per plan.md:2137

**Checkpoint**: All documentation complete, deployment ready

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all user stories
- **US3 Ingestion (Phase 3)**: Depends on Foundational - BLOCKS US1 and US6
- **US1 Basic Chat (Phase 4)**: Depends on Foundational + US3
- **US2 Selection Mode (Phase 5)**: Depends on Foundational only (independent of US1)
- **US6 Evaluation (Phase 6)**: Depends on Foundational + US3
- **US4 Persistence (Phase 7)**: Depends on Foundational + US1 (optional)
- **US5 Health (Phase 8)**: Depends on Foundational (optional)
- **Deployment (Phase 9)**: Depends on desired user stories complete
- **Polish (Phase 10)**: Depends on all desired user stories

### User Story Dependencies (Critical Path)

```
Foundational (Phase 2) → US3 Ingestion (Phase 3) → US1 Basic Chat (Phase 4) → MVP ✅
                            ↓
                         US6 Evaluation (Phase 6) → Validation ✅

Independent paths:
Foundational (Phase 2) → US2 Selection Mode (Phase 5)
Foundational (Phase 2) + US1 → US4 Persistence (Phase 7)
Foundational (Phase 2) → US5 Health (Phase 8)
```

### Within Each User Story

- Backend before frontend (for US1)
- Models → Services → Endpoints → Integration
- Tests can run in parallel if marked [P]
- Story complete before moving to next priority

### Parallel Opportunities

**Setup Phase**:
- T006, T007 (frontend dirs) parallel with T001-T005 (backend dirs)

**Foundational Phase**:
- T010 (embedding service) parallel with T009 (Qdrant client)

**US3 Ingestion**:
- T013, T014 (chunker components) parallel

**US1 Backend**:
- T024, T025 (schemas, llm service) parallel
- T031 (health) parallel with T024-T030

**US1 Frontend**:
- T035, T036, T037, T038 (all React components/hooks) parallel

**US2 Selection Mode**:
- T041, T042, T043 parallel

**US6 Evaluation**:
- T048, T049, T050 (test files) parallel

**US4 Persistence**:
- T059-T063 (all Postgres methods) parallel

**Deployment**:
- T072-T076 (all config files) parallel

---

## Parallel Example: User Story 1 Backend

```bash
# Launch in parallel (different files, no dependencies):
Task T024: Create backend/app/models/schemas.py
Task T025: Create backend/app/services/llm.py
Task T031: Create backend/app/api/health.py

# Then sequentially (dependencies):
Task T026: Create rag_service.py (needs schemas)
Task T027: Implement RAG pipeline (needs rag_service)
Task T028: Create chat.py endpoint (needs rag_service)
```

---

## Implementation Strategy

### MVP First (US3 + US1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL)
3. Complete Phase 3: US3 Ingestion (FOUNDATIONAL - indexes content)
4. Complete Phase 4: US1 Basic Chat
5. **STOP and VALIDATE**: Test chat end-to-end, verify citations
6. Deploy/demo MVP

**MVP Delivers**: Functional RAG chatbot with 21 docs indexed, answers questions with citations

### Incremental Delivery

1. **MVP**: US3 + US1 → Test → Deploy (core value)
2. **+Validation**: US6 Evaluation → Verify ≥90% accuracy → Deploy
3. **+Selection**: US2 Selection Mode → Test independently → Deploy
4. **+Persistence**: US4 Conversation History → Test independently → Deploy
5. **+Monitoring**: US5 Health Checks → Test independently → Deploy

Each increment adds value without breaking previous features.

### Parallel Team Strategy

With 3 developers after Foundational + US3 complete:

1. **Developer A**: US1 Basic Chat (MVP critical path)
2. **Developer B**: US6 Evaluation (validates US1)
3. **Developer C**: US2 Selection Mode (independent feature)

After MVP validated:
- **Developer A**: US4 Persistence
- **Developer B**: US5 Health + Deployment
- **Developer C**: Polish + Documentation

---

## Task Analysis per User Request

### (1) Atomic Tasks - ONE thing with ONE acceptance criterion?

**YES** - Examples:
- ✅ T013: "Create chunker.py with DocumentChunker class" - ONE file, ONE class
- ✅ T015: "Implement parse_markdown() with H2/H3 detection" - ONE method, ONE feature
- ✅ T041: "Create useTextSelection.ts with mouseup detection" - ONE hook, ONE event

**NO VIOLATIONS** - All tasks do exactly one thing

### (2) Sized Right (15-30 minutes)?

**YES** - Breakdown:
- **Setup tasks (T001-T007)**: 5-10 min each (mkdir, touch files, copy config)
- **Foundational (T008-T012)**: 15-20 min each (single class/file from plan)
- **Implementation (T013-T071)**: 15-30 min each (method, endpoint, component)
- **Deployment (T072-T079)**: 10-15 min each (Dockerfile, config files)
- **Polish (T080-T086)**: 10-20 min each (documentation, final checks)

**Potentially too large**:
- T037 (ChatWidget.tsx): 200+ lines - **KEEP**: Complex but atomic (one component, copied from plan)
- T051 (evaluate.py): 90 lines - **KEEP**: Atomic evaluation script

**ALL WITHIN RANGE**: Plan provides copy-paste ready code, minimal adaptation needed

### (3) Can Each Be Reviewed Independently?

**YES** - Each task:
- ✅ Has clear file path
- ✅ Creates/modifies ONE file (or related __init__.py)
- ✅ Has explicit code from plan.md to reference
- ✅ Can be reviewed via single file diff

**Example**: T024 (schemas.py) can be reviewed without seeing T025 (llm.py)

### (4) Tasks to Split or Combine?

**SPLIT RECOMMENDATIONS**:
- **NONE** - All tasks already atomic per plan's micro-tasks (2-5 min each grouped)

**COMBINE RECOMMENDATIONS**:
- **T001 + T002 + T003**: Could combine into "Create full backend structure" (20 min)
  - **DECISION**: KEEP separate - enables parallel frontend (T006-T007)

- **T048 + T049 + T050**: Could combine into "Create all test files" (25 min)
  - **DECISION**: KEEP separate - marked [P] for parallel execution

**FINAL**: NO changes needed - granularity optimal for 15-30 min atomic tasks

### (5) Tasks to Add or Remove?

**ADD RECOMMENDATIONS**:

1. **T012.5**: Test Qdrant collection metadata schema
   - **Reasoning**: Validates FR-013 (metadata includes heading_path, token_count)
   - **DECISION**: SKIP - covered by T023 (ingestion verification)

2. **T034.5**: Test out-of-scope question handling
   - **Reasoning**: Edge case from spec.md:126
   - **DECISION**: SKIP - covered by US6 Evaluation (T052-T053)

3. **T040.5**: Test source citation navigation
   - **Reasoning**: Acceptance scenario spec.md:31 (click citation → navigate)
   - **DECISION**: ADD below

4. **T047.5**: Test selection mode with multi-paragraph text
   - **Reasoning**: Edge case spec.md:123
   - **DECISION**: SKIP - covered by T041 (8000 char limit)

**REMOVE RECOMMENDATIONS**:
- **NONE** - All tasks map to plan.md phases or spec.md requirements

**FINAL ADDITIONS**:

```markdown
- [ ] T040.5 [US1] Test citation navigation: click source link, verify page navigation per spec.md:31
```

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story independently completable and testable
- All code referenced from plan.md lines (copy-paste ready)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- **CRITICAL PATH**: Setup → Foundational → US3 → US1 = MVP
