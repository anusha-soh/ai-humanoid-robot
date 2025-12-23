---
id: 001
title: HF Spaces Deployment Specification
stage: spec
date: 2025-12-22
surface: agent
model: claude-opus-4-5-20251101
feature: 003-hf-spaces-deploy
branch: 003-hf-spaces-deploy
user: developer
command: /sp.specify
labels: ["deployment", "huggingface", "docker", "backend"]
links:
  spec: specs/003-hf-spaces-deploy/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/003-hf-spaces-deploy/spec.md
  - specs/003-hf-spaces-deploy/checklists/requirements.md
tests:
  - Specification quality checklist validated (16/16 items passed)
---

## Prompt

Deploy the RAG chatbot backend to Hugging Face Spaces instead of Railway.

Feature name: 003-huggingface-spaces-deployment

Context:
- RAG chatbot with FastAPI backend (streaming chat, conversation persistence)
- External services already set up: Qdrant Cloud (vector DB), Neon PostgreSQL, Google Gemini API
- Frontend deployed on Vercel
- Railway deployment having issues, switching to HF Spaces

Requirements:
- Use Docker SDK on Hugging Face Spaces
- Port 7860 (HF Spaces requirement)
- Non-root user in container (HF Spaces security requirement)
- Connect to existing external services (Qdrant Cloud, Neon Postgres, Gemini API)
- Environment variables configured as HF Secrets
- Update frontend on Vercel to use new backend URL (https://username-rag-chatbot.hf.space)
- Provide step-by-step deployment guide
- Health endpoint must return proper status for HF monitoring
- SSE streaming must work through HF Spaces infrastructure

Success criteria:
- Backend deploys successfully to HF Spaces
- Health check returns HTTP 200 with all services healthy
- Chat endpoint streams responses correctly
- Frontend on Vercel can communicate with HF Spaces backend
- Deployment is repeatable with clear documentation

## Response snapshot

Created comprehensive feature specification for Hugging Face Spaces backend deployment:

- **4 User Stories**: Deploy to HF Spaces (P1), Configure Secrets (P1), Connect Frontend (P2), Documentation (P2)
- **17 Functional Requirements**: Covering Dockerfile, README.md, secrets, CORS, health endpoint, SSE streaming
- **11 Success Criteria**: Measurable outcomes for deployment verification
- **7 Edge Cases**: Sleep/wake, service failures, build errors, permissions

Key differences from Railway:
- Port 7860 (not dynamic)
- Docker SDK (not auto-detect)
- Non-root user required
- README.md with YAML frontmatter

Quality checklist passed: 16/16 items validated.

## Outcome

- Impact: Feature specification created, ready for implementation planning
- Tests: Quality checklist validated (16/16 passed)
- Files: spec.md, checklists/requirements.md
- Next prompts: /sp.plan to generate implementation plan
- Reflection: Spec-driven approach ensures all requirements captured before implementation

## Evaluation notes (flywheel)

- Failure modes observed: None - user confirmed all external services ready
- Graders run and results (PASS/FAIL): Quality checklist PASS
- Prompt variant (if applicable): Standard /sp.specify flow
- Next experiment (smallest change to try): Proceed to /sp.plan
