---
description: "Implementation tasks for deploying backend to Hugging Face Spaces"
---

# Tasks: Deploy Backend to Hugging Face Spaces

**Input**: Design documents from `/specs/001-deploy-backend-hf/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/, quickstart.md

**Tests**: No automated tests requested - manual API testing only (see success criteria in each phase)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `- [ ] [ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Include exact file paths in descriptions

## Path Conventions

- Backend: `rag-backend/chatbot/` (deployment source)
- Frontend: `my-website/` (URL updates)
- Deployment files to create in: `rag-backend/chatbot/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Prepare repository for deployment, verify current state

- [x] T001 Verify .gitignore excludes .env files (check .gitignore at repository root)
- [x] T002 [P] Create backup of current app.py configuration in rag-backend/chatbot/app.py.backup
- [x] T003 [P] Document current localhost:8001 backend URL for rollback reference

**Checkpoint**: Repository prepared for deployment file creation

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create all deployment configuration files - MUST be complete before deployment

**⚠️ CRITICAL**: No user story work can begin until these deployment files exist

- [ ] T004 Create requirements.txt with pinned dependencies in rag-backend/chatbot/requirements.txt
- [ ] T005 [P] Create Dockerfile for HF Spaces (port 7860) in rag-backend/chatbot/Dockerfile
- [ ] T006 [P] Create .env.example with placeholder values in rag-backend/chatbot/.env.example
- [ ] T007 [P] Create README.md with API documentation in rag-backend/chatbot/README.md
- [ ] T008 Update CORS configuration to include Vercel production domain in rag-backend/chatbot/app.py (lines 14-20)

**Checkpoint**: All deployment files created - ready to deploy to Hugging Face Spaces

---

## Phase 3: User Story 1 - Production Backend Accessible (Priority: P1) 🎯 MVP

**Goal**: Deploy backend to Hugging Face Spaces and make it accessible from production Vercel frontend

**Independent Test**:
1. Visit production Vercel site
2. Click chatbot → send question → verify response
3. Click "اردو" button → verify Urdu translation appears
4. Check browser console → verify no CORS errors

### Implementation for User Story 1

- [ ] T009 [US1] Create new Hugging Face Space with Docker SDK (name: humanoid-robotics-chatbot)
- [ ] T010 [US1] Upload deployment files to HF Space (app.py, config.py, requirements.txt, Dockerfile, .env.example)
- [ ] T011 [US1] Trigger initial build and monitor logs for errors in HF Spaces dashboard
- [ ] T012 [US1] Verify health endpoint responds at https://[space-url]/health
- [ ] T013 [US1] Test chat endpoint with curl: POST https://[space-url]/chat
- [ ] T014 [US1] Test translation endpoint with curl: POST https://[space-url]/translate
- [ ] T015 [US1] Test CORS preflight from browser console on Vercel site
- [ ] T016 [US1] Update frontend translation component URL in my-website/src/theme/DocItem/Content/index.js (line 69)
- [ ] T017 [US1] Commit frontend changes and push to trigger Vercel deployment
- [ ] T018 [US1] Wait for Vercel deployment to complete (2-3 minutes)
- [ ] T019 [US1] End-to-end test: Visit production site, use chatbot and translation features

**Success Criteria**:
- ✅ HF Space shows "Running" status
- ✅ Health endpoint returns {"status": "ok", "version": "1.0"}
- ✅ Chatbot responds with relevant answers from book content
- ✅ Translation completes within 10 seconds
- ✅ No CORS errors in browser console
- ✅ Frontend successfully calls production backend URL

**Checkpoint**: Backend is publicly accessible and frontend uses production URL - MVP complete!

---

## Phase 4: User Story 2 - Secure Configuration Management (Priority: P1)

**Goal**: Configure environment variables securely in HF Spaces, verify no secrets exposed

**Independent Test**:
1. View HF Space files → verify no API keys in code
2. Check HF Space logs → verify no keys logged
3. Clone HF Space repository → verify no secrets committed
4. Backend still connects to Cohere and Qdrant successfully

### Implementation for User Story 2

- [ ] T020 [US2] Navigate to HF Space Settings → Repository secrets
- [ ] T021 [US2] Add COHERE_API_KEY secret in HF Spaces settings
- [ ] T022 [P] [US2] Add QDRANT_URL secret in HF Spaces settings
- [ ] T023 [P] [US2] Add QDRANT_API_KEY secret in HF Spaces settings
- [ ] T024 [P] [US2] Add COLLECTION_NAME secret (value: "book") in HF Spaces settings
- [ ] T025 [US2] Factory reboot HF Space to apply environment variables
- [ ] T026 [US2] Monitor Space logs for successful Cohere connection
- [ ] T027 [US2] Monitor Space logs for successful Qdrant connection
- [ ] T028 [US2] Verify config.py reads environment variables correctly (check logs)
- [ ] T029 [US2] Test chat endpoint to confirm Cohere API working
- [ ] T030 [US2] Test translation endpoint to confirm Qdrant connection working
- [ ] T031 [US2] Search all files in HF Space for any hardcoded API keys (should find none)
- [ ] T032 [US2] Review recent Space logs to ensure no API keys appear in output

**Success Criteria**:
- ✅ All 4 environment variables configured in HF Spaces secrets
- ✅ Backend connects to Cohere and Qdrant using env vars
- ✅ No API keys visible in any committed files
- ✅ No API keys visible in Space logs
- ✅ Config changes can be made without code changes

**Checkpoint**: All secrets secured in environment variables, no exposure in code or logs

---

## Phase 5: User Story 3 - Reliable Service Availability (Priority: P2)

**Goal**: Ensure backend maintains 99% uptime with fast response times

**Independent Test**:
1. Monitor /health endpoint for 24 hours
2. Calculate uptime percentage
3. Measure average response time for chat queries
4. Test error recovery (intentionally cause error, verify auto-restart)

### Implementation for User Story 3

- [ ] T033 [US3] Set up HF Space monitoring dashboard (built-in)
- [ ] T034 [US3] Configure auto-restart on crash (HF Spaces default behavior)
- [ ] T035 [US3] Verify health endpoint returns 200 OK consistently
- [ ] T036 [US3] Test concurrent requests (send 5 chat queries simultaneously)
- [ ] T037 [US3] Measure average response time over 10 requests (should be <3 seconds)
- [ ] T038 [US3] Measure translation time over 5 requests (should be <10 seconds)
- [ ] T039 [US3] Test error recovery: Intentionally trigger error (invalid env var name in config)
- [ ] T040 [US3] Verify Space auto-restarts after error
- [ ] T041 [US3] Document uptime statistics from HF Spaces dashboard (24-hour period)
- [ ] T042 [US3] Create monitoring checklist for weekly health checks

**Success Criteria**:
- ✅ Service maintains 99% uptime over 7-day period
- ✅ Average API response time <3 seconds
- ✅ Average translation time <10 seconds
- ✅ Concurrent queries (5 simultaneous) all respond within 5 seconds
- ✅ Service auto-restarts after errors

**Checkpoint**: Backend demonstrates reliable 24/7 availability with acceptable performance

---

## Phase 6: User Story 4 - Easy Troubleshooting (Priority: P3)

**Goal**: Enable quick problem identification through clear logs and documentation

**Independent Test**:
1. Intentionally cause an error (e.g., invalid API key)
2. Check HF Space logs
3. Verify error message clearly indicates the problem
4. Fix error and confirm resolution via logs

### Implementation for User Story 4

- [ ] T043 [US4] Review current logging in app.py (lines 219, 227, 253, 299)
- [ ] T044 [US4] Enhance error logging to include timestamps and endpoint names in rag-backend/chatbot/app.py
- [ ] T045 [US4] Test error logging: Temporarily set invalid Cohere key
- [ ] T046 [US4] Verify error log shows clear message: "Cohere connection failed"
- [ ] T047 [US4] Test error logging: Temporarily set invalid Qdrant URL
- [ ] T048 [US4] Verify error log shows clear message: "Qdrant connection timeout"
- [ ] T049 [US4] Document common errors and solutions in rag-backend/chatbot/README.md
- [ ] T050 [US4] Create troubleshooting section in README.md with log examples
- [ ] T051 [US4] Test health endpoint includes service connection status
- [ ] T052 [US4] Document how to access HF Space logs in README.md

**Success Criteria**:
- ✅ Error messages clearly indicate problem and affected endpoint
- ✅ Logs include timestamps for all errors
- ✅ README.md includes troubleshooting guide
- ✅ Common errors documented with solutions
- ✅ Health endpoint shows connection status for Cohere and Qdrant

**Checkpoint**: Comprehensive logging and documentation enable quick issue resolution

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final cleanup, optimization, and documentation

- [ ] T053 [P] Remove unused my-website/backend/server.js (old Gemini backend)
- [ ] T054 [P] Update project README.md with deployment documentation (repository root)
- [ ] T055 [P] Document HF Space URL in project documentation
- [ ] T056 [P] Create deployment runbook for future updates
- [ ] T057 Verify all acceptance criteria from spec.md are met
- [ ] T058 Perform final end-to-end test on production site
- [ ] T059 Document lessons learned in specs/001-deploy-backend-hf/lessons-learned.md
- [ ] T060 Close feature branch and merge to main

**Success Criteria**:
- ✅ All user stories (US1-US4) fully implemented and tested
- ✅ All acceptance scenarios from spec.md pass
- ✅ Documentation complete and accessible
- ✅ Production site fully functional

---

## Dependencies & Execution Order

### Story Dependency Graph

```
Phase 1 (Setup)
     ↓
Phase 2 (Foundational) ← BLOCKING
     ↓
Phase 3 (US1: Production Backend) ← MVP ← INDEPENDENT
     ↓ (optional dependency)
Phase 4 (US2: Secure Config) ← INDEPENDENT
     ↓ (optional dependency)
Phase 5 (US3: Reliability) ← INDEPENDENT (needs US1 deployed)
     ↓ (optional dependency)
Phase 6 (US4: Troubleshooting) ← INDEPENDENT (needs US1 deployed)
     ↓
Phase 7 (Polish) ← Waits for US1-US4
```

### Parallel Execution Opportunities

**Within Phase 1** (can run in parallel):
- T002, T003 (different files)

**Within Phase 2** (can run in parallel):
- T005, T006, T007 (independent file creation)

**Within Phase 4** (can run in parallel):
- T022, T023, T024 (adding different secrets)

**Within Phase 7** (can run in parallel):
- T053, T054, T055, T056 (documentation tasks)

**User Stories** (can implement in parallel after Phase 2):
- US1 must complete first (deploys backend)
- US2 can run alongside or after US1 (adds secrets)
- US3 requires US1 deployed (monitors live service)
- US4 requires US1 deployed (tests logging on live service)

### Recommended MVP Scope

**Minimum Viable Product** = Phase 1 + Phase 2 + Phase 3 (US1)

This delivers:
- ✅ Backend deployed to HF Spaces
- ✅ Production Vercel site uses backend
- ✅ CORS configured correctly
- ✅ Chatbot and translation working
- ✅ Basic deployment files created

**After MVP**, add incrementally:
- Phase 4 (US2): Secure secrets (P1 - critical)
- Phase 5 (US3): Monitor reliability (P2)
- Phase 6 (US4): Enhance logging (P3)
- Phase 7: Polish

---

## Implementation Strategy

### Approach: Sequential MVP with Optional Enhancements

1. **Phase 1-3 (MVP)**: Deploy working backend
   - Focus: Get production site functional
   - Success: Chatbot and translation work from Vercel
   - Time: ~1-2 hours

2. **Phase 4 (Security)**: Lock down secrets
   - Focus: Environment variable configuration
   - Success: No exposed API keys
   - Time: ~30 minutes

3. **Phase 5 (Reliability)**: Monitor uptime
   - Focus: Performance validation
   - Success: 99% uptime confirmed
   - Time: ~24 hours (monitoring period)

4. **Phase 6 (Troubleshooting)**: Enhance logs
   - Focus: Operational readiness
   - Success: Clear error messages
   - Time: ~1 hour

5. **Phase 7 (Polish)**: Clean up
   - Focus: Documentation and cleanup
   - Success: Complete and documented
   - Time: ~30 minutes

### Total Estimated Effort

- **MVP (US1)**: 1-2 hours
- **Security (US2)**: 30 minutes
- **Reliability (US3)**: 24 hours monitoring + 1 hour setup
- **Troubleshooting (US4)**: 1 hour
- **Polish**: 30 minutes
- **Total Active Work**: ~4-5 hours
- **Total with Monitoring**: ~28-29 hours (includes 24h uptime monitoring)

---

## Task Validation Checklist

Format compliance:
- ✅ All tasks use `- [ ] [ID]` format
- ✅ Parallelizable tasks marked with [P]
- ✅ User story tasks labeled with [US1], [US2], [US3], [US4]
- ✅ All tasks include exact file paths
- ✅ Task IDs sequential (T001-T060)

Organization compliance:
- ✅ Phase 1: Setup (3 tasks)
- ✅ Phase 2: Foundational - BLOCKING (5 tasks)
- ✅ Phase 3: US1 (11 tasks) - MVP
- ✅ Phase 4: US2 (13 tasks) - P1 critical
- ✅ Phase 5: US3 (10 tasks) - P2
- ✅ Phase 6: US4 (10 tasks) - P3
- ✅ Phase 7: Polish (8 tasks)
- ✅ Total: 60 tasks

Independent testing:
- ✅ Each user story phase includes "Independent Test" section
- ✅ Each phase includes "Success Criteria" checklist
- ✅ Checkpoints verify story completion

---

## Summary

**Total Tasks**: 60
- Setup: 3
- Foundational (blocking): 5
- US1 (P1 - MVP): 11
- US2 (P1): 13
- US3 (P2): 10
- US4 (P3): 10
- Polish: 8

**Parallel Opportunities**: 11 tasks marked [P]
- Phase 1: 2 tasks
- Phase 2: 3 tasks
- Phase 4: 3 tasks
- Phase 7: 4 tasks

**MVP Scope**: Phase 1-3 (19 tasks, ~1-2 hours)

**Ready for Execution**: All tasks have clear descriptions, file paths, and acceptance criteria
