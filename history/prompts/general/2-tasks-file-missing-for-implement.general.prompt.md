---
id: 2
title: Tasks file missing for implement
stage: general
date: 2025-12-10
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: main
user: unknown
command: /sp.implement
labels: ["implement", "error", "tasks-missing"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:

tests:

---

## Prompt

/sp.implement

## Response snapshot

The `tasks.md` file is missing. Please run `/sp.tasks` first to generate the task list.

## Outcome

- ✅ Impact: User was informed about the missing tasks file and advised on the next step.
- 🧪 Tests: N/A
- 📁 Files: No files were modified, one PHR file created.
- 🔁 Next prompts: User needs to run /sp.tasks.
- 🧠 Reflection: The tasks file was not found, so the implementation process could not proceed.

## Evaluation notes (flywheel)

- Failure modes observed: Tasks file missing.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
