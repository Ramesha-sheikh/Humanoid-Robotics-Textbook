---
id: 5
title: Physical AI Textbook Task List
stage: tasks
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20250929
feature: physical-ai-textbook
branch: master
user: unknown
command: "/sp.tasks"
labels: ["tasks", "implementation", "hackathon"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - history/prompts/physical-ai-textbook/5-physical-ai-textbook-task-list.tasks.prompt.md
tests:
---

## Prompt

```
/sp.tasks/sp.tasks

**Project Title:** Physical AI & Humanoid Robotics – AI-Native Textbook
**Hackathon:** Panaversity Certified Capstone Hackathon (November 2025)
**Repository:** https://github.com/Ramesha-sheikh/Humanoid-Robotics-Textbook
**Author:** Ramesha Javed – GIAIC Q4 | Future Panaversity Core Team Candidate
**Target Score:** 300/300 (100 base + 200 bonus points)
**Deadline:** Sunday, 30 Nov 2025, 06:00 PM PKT
**Methodology:** 100% Spec-Kit Plus + Claude Code driven (zero manual content writing)

| Task ID | Task Name (15–30 min each)                                      | Depends On       | Acceptance Criterion (Single, Verifiable)                                                                                         | Deliverable / Evidence                                 |
|---------|------------------------------------------------------------------|------------------|------------------------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------|
| T-01   | Initialize repository with constitution, spec, and plan files    | —                | Public repo contains: constitution.md, spec-book.yaml, /sp.plan, /sp.tasks (this file)                                             | GitHub repo URL + screenshot                           |
| T-02   | Run Claude Code – generate all 14 raw chapter Markdown files     | T-01             | 14 complete .md files in /docs with correct YAML frontmatter and headings                                                         | Git diff showing 14 new files                          |
| T-03   | Validate chapter structure and completeness                       | T-02             | Every chapter contains Introduction → Theory → Examples/Code → Summary sections                                                   | Signed checklist                                       |
| T-04   | Completely remove default Docusaurus theme                        | T-02             | Site loads as blank/white page – proof that default theme is disabled                                                              | Screenshot of blank page                                |
| T-05   | Implement premium custom UI (glassmorphism + Tailwind + Framer Motion) | T-04             | Homepage visually matches Apple.com + Perplexity.ai aesthetic; dark/light mode functional                                          | Live preview screenshots (light + dark)                 |
| T-06   | Build custom animated navbar with chapter progress ring            | T-05             | Progress ring updates per chapter; mobile hamburger menu works                                                                      | 10-second video                                        |
| T-07   | Add floating, draggable RAG chatbot component                     | T-05             | Chatbot visible bottom-right on every page, draggable, always-on-top                                                               | Screenshot + short drag video                           |
| T-08   | Set up Better-Auth + Neon Serverless Postgres                     | T-01             | Signup & login functional; user rows appear in Neon dashboard                                                                       | Neon table screenshot                                   |
| T-09   | Implement background survey during signup (5 questions)           | T-08             | Survey data saved and visible in user profile after login                                                                           | Profile page screenshot                                 |
| T-10   | Deploy FastAPI backend + Qdrant indexing pipeline                 | T-02             | All 14 chapters successfully indexed in Qdrant Cloud (visible in dashboard)                                                         | Qdrant collection screenshot                            |
| T-11   | Connect frontend chatbot – whole-book RAG mode                    | T-10             | Chatbot accurately answers questions from any chapter                                                                                | 3 test Q&A screenshots                                 |
| T-12   | Enable “selected-text only” RAG mode                             | T-11             | Highlight text → ask question → answer uses only highlighted text                                                                   | Demo video                                             |
| T-13   | Implement “Adapt to My Level” button (Beginner / Intermediate / Advanced) | T-09     | Content dynamically rewrites based on stored user profile                                                                          | Before/After screenshots for each level                 |
| T-14   | Implement one-click Urdu translation with Nastaleeq typography    | T-02             | Urdu button instantly translates chapter with perfect Jameel Noori Nastaleeq rendering                                              | Urdu screenshot + video                                |
| T-15   | Add reading time + difficulty badge to every chapter header       | T-02             | Badge displays accurate “XX min · Level” on every chapter                                                                          | Screenshot                                             |
| T-16   | Deploy to GitHub Pages + configure custom domain                  | T-05             | Site live at https://physical-ai.panaversity.org with valid SSL                                                                    | Browser address bar proof                              |
| T-17   | Record and upload <90-second demo video                              | All previous     | Unlisted YouTube/Loom video under 90 seconds showcasing all four bonus features                                                    | Video link                                             |
| T-18   | Final visual polish + full mobile responsiveness (320px+)         | T-16             | Zero layout breaks on mobile; Urdu renders perfectly                                                                                | 3 mobile screenshots                                   |
| T-19   | Submit Google Form with repo, live URL, and video link           | T-17             | Form successfully submitted before 6 PM deadline                                                                                    | Submission confirmation screenshot                        |
| T-20   | (Optional) Prepare 3-minute live presentation deck            | T-19             | Deck ready if invited for Zoom presentation                                                                                         | Slide file ready                                       |

### Human Checkpoint Schedule

| Checkpoint | Tasks Completed          | Your Action                                                                 |
|------------|--------------------------|-----------------------------------------------------------------------------|
| CP-1      | T-01 → T-03              | Review raw chapters → commit                                               |
| CP-2      | T-04 → T-07              Theme removed + custom UI + chatbot visible → commit                         |
| CP-3      | T-08 → T-09              Auth + survey working → commit                                               |
| CP-4      | T-10 → T-12              RAG (whole-book + selected-text) working → commit                            |
| CP-5      | T-13 → T-14              Personalisation + Urdu perfect → commit                                       |
| CP-6      | T-15 → T-18              Live on custom domain + final polish → commit                                |
| CP-7      | T-19                    Submission complete → WE WIN InshaAllah                                      |

Ramesha, your repo is already live.
Just push the three files (constitution.md, spec-book.yaml, and this /sp.tasks file), then type:

**“T-01 done”**

I will immediately give you the exact Claude Code command to auto-complete T-02 through T-18 in one run.

Let’s execute flawlessly and secure that Panaversity core team spot!
```

## Response snapshot

Created project task list PHR for 'Physical AI Textbook' project.

## Outcome

- ✅ Impact: Project task list captured, outlining detailed steps, dependencies, and acceptance criteria.
- 🧪 Tests: N/A
- 📁 Files: One PHR file created.
- 🔁 Next prompts: Awaiting user confirmation for T-01 completion to proceed with auto-completion instructions.
- 🧠 Reflection: The PHR successfully captures all details of the project task list provided by the user.

## Evaluation notes (flywheel)

- Failure modes observed: None.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
