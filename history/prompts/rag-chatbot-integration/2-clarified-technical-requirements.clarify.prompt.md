---
id: 2
title: Clarified Technical Requirements and Constraints
stage: spec
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5
feature: rag-chatbot-integration
branch: 001-responsive-ui
user: Ramesha Javed
command: /sp.clarify
labels: ["clarification", "technical-constraints", "mobile-responsive", "state-machine", "gemini-flash"]
links:
  spec: specs/rag-chatbot-integration/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/rag-chatbot-integration/spec.md (updated with clarifications)
tests:
  - none (specification refinement stage)
---

## Prompt

User ran `/sp.clarify` and provided critical technical clarifications:

```
# /sp.clarify — Final Clarifications

- LangGraph banned → replaced with simple custom lightweight state machine
- Gemini 2.0 Flash is mandatory (fastest response <400ms)
- OpenAI Agents SDK sirf compatibility ke liye use hoga (via LiteLLM wrapper)
- Highlight mode = send only selected text chunks as context (zero leakage)
- Mobile responsive mandatory
```

## Response snapshot

Updated specification document with 5 critical clarifications:

**1. State Management Architecture**
- ❌ LangGraph/LangChain completely banned
- ✅ Simple custom lightweight state machine
- Updated FR-012 and TC-004 to reflect this constraint

**2. LLM Performance Requirement**
- Gemini 2.0 Flash is mandatory (non-negotiable)
- Target response latency: <400ms for first token
- Updated NFR-001 from "3 seconds" to "400ms" (MUST requirement)
- Updated TC-003 to emphasize mandatory nature

**3. OpenAI Agents SDK Usage**
- Clarified it's ONLY for compatibility layer
- Must use LiteLLM wrapper (not as primary SDK)
- Added FR-013 and TC-005 to document this constraint

**4. Highlight Mode Zero Leakage**
- Enhanced FR-004 and added FR-004a
- Explicitly states: send ONLY selected text chunks as context
- No vector database retrieval in Highlight Mode
- Added acceptance scenario to User Story 3

**5. Mobile Responsiveness (Mandatory)**
- Added new User Story 6 (Priority P1) - Mobile Responsive Interface
- Added FR-017 and FR-018 for mobile requirements
- Added TC-009 requiring mobile-responsive UI
- Added 4 mobile-specific edge cases (keyboard, touch selection, small screens, device rotation)
- Updated NFR-005 to include mobile browsers

**Open Questions Updated:**
- Moved 5 clarified items to "✅ CLARIFIED" section
- 6 remaining questions for planning phase

## Outcome

- ✅ Impact: Specification now has crystal-clear technical constraints eliminating ambiguity about LangGraph, Gemini requirements, OpenAI SDK usage, Highlight Mode behavior, and mobile responsiveness
- 🧪 Tests: Mobile acceptance scenarios added with specific viewport breakpoints and touch target requirements
- 📁 Files: Updated `specs/rag-chatbot-integration/spec.md` (added 3 new requirements, 1 user story, enhanced 6 existing requirements, added 4 mobile edge cases)
- 🔁 Next prompts: `/sp.plan` to create architectural design with state machine architecture and mobile-first approach
- 🧠 Reflection: These clarifications prevent scope creep and technical drift. The <400ms latency requirement with Gemini 2.0 Flash is aggressive but achievable. Zero-leakage Highlight Mode is architecturally simpler (no retrieval) but requires careful frontend text extraction. Mobile-first is critical for educational content.

## Evaluation notes (flywheel)

- Failure modes observed: None - clarifications successfully integrated
- Graders run and results (PASS/FAIL): Manual review - PASS (all clarifications captured, no conflicts with existing requirements)
- Prompt variant (if applicable): User provided clarifications in mix of English and Urdu (sirf = only), successfully interpreted
- Next experiment (smallest change to try): Consider documenting the custom state machine states/transitions in planning phase (e.g., Idle → Retrieving → Generating → Streaming → Complete)
