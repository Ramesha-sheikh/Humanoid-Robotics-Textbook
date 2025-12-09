---
id: 0001
title: Physical AI & Humanoid Robotics Textbook Constitution
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20251101
feature: none
branch: master
user: user
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
null
tests:
null
---

## Prompt

/sp.constitution# Physical AI & Humanoid Robotics Textbook Constitution v1.0.0

## 1. Vision & Why Physical AI Matters
We are writing the world’s first beginner-to-advanced textbook that teaches students can use to go from zero knowledge to building a fully autonomous conversational humanoid robot using ROS 2, Gazebo, NVIDIA Isaac Sim, and Vision-Language-Action (VLA) models.
The future of work = Humans + AI Agents + Humanoid Robots working together. This book is the bridge.

## 2. Book Title (Final)
**Physical AI & Humanoid Robotics: Bridging Digital Intelligence with the Physical World**

## 3. Complete Book Structure (Strictly follows hackathon 13-week outline)
- Introduction – Why Physical AI Matters (Weeks 1-2)
- Module 1 – The Robotic Nervous System (ROS 2) → Weeks 3-5
- Module 2 – The Digital Twin (Gazebo & Unity) → Weeks 6-7
- Module 3 – The AI-Robot Brain (NVIDIA Isaac™) → Weeks 8-10
- Module 4 – Vision-Language-Action (VLA) → Weeks 11-13 + Capstone
- Appendices – Installation Guides, Hardware Recommendations, Sim-to-Real Transfer

## 4. Mandatory Chapter Format (Every chapter MUST follow this exact order)
1. Learning Objectives
2. Theory & Real-World Motivation (Tesla Optimus, Figure 01, Atlas examples)
3. Core Concepts Explained (beginner-friendly)
4. Step-by-Step Code (Python 3.11+, rclpy, type hints, Pydantic models)
5. Line-by-Line Code Breakdown (Roman Urdu comments allowed)
6. Simulation Walkthrough (Gazebo / Isaac Sim screenshots + commands)
7. Common Errors & Debugging Tips
8. Mini-Project / Exercise
9. Quiz (5 MCQs)
10. Further Reading & Video Links

## 5. Code & Simulation Standards (Non-Negotiable)
- Python ≥ 3.11 with type hints everywhere
- ROS 2 Humble or Iron
- All ROS packages must include launch files and proper CMakeLists.txt
- Gazebo Garden or Ignition
- NVIDIA Isaac Sim 2023.1+ (free academic license)
- All code snippets must run in Docker when possible
- URDF/XACRO files must be valid and visualised with Mermaid or RViz screenshots

## 6. Diagramming Rules
- Flowcharts → Mermaid only
- URDF tree → Mermaid hierarchy diagrams
- Architecture → draw.io exported PNG with consistent blue-orange theme

## 7. Writing Style & Voice
- Professional but super friendly (jaise dost samjha raha ho)
- Heavy use of real humanoid examples
- Roman Urdu code comments allowed for clarity
- Zero jargon without immediate definition
- Every concept linked back to “How will this help me control a real humanoid?”

## 8. AI Collaboration Loop (Human + AI)
- Human (Ramesha) always stays in the loop
- Claude Code + Spec-Kit Plus used for every chapter
- Never fully autonomous generation
- Every generated file reviewed and edited by human before commit

## 9. Docusaurus Formatting Rules
- All chapters live in /docs folder
- Sidebar automatically generated via sidebars.js
- Dark mode enabled by default
- Search enabled
- Mermaid plugin active
- Custom CSS for code blocks (Dracula theme)

## 10. Embedded RAG Chatbot (Mandatory Hackathon Deliverable)
- Must answer questions on full book content
- Must answer questions on user-selected text only
- Backend: FastAPI + OpenAI Agents SDK + Neon Serverless Postgres + Qdrant Cloud Free Tier
- Frontend: iframe or custom React widget inside every page
- Chatbot must show source chunk + page link

## 11. Quality Assurance Checklist (10-point)
[ ] Human read & edited
[ ] Code runs in fresh Docker container
[ ] Screenshots match current Isaac Sim/Gazebo version
[ ] All Mermaid diagrams render
[ ] RAG chatbot tested with 5 sample questions
[ ] Mobile responsive
[ ] No broken links
[ ] Sidebar order correct
[ ] Quiz answers provided
[ ] Learning objectives achieved

## 12. Non-Negotiable Rules
**We ALWAYS**
- Put safety first (never disable ROS 2 safety controllers in examples)
- Show both simulation and sim-to-real path
- Credit every external resource
- Use type hints
- Include line-by-line explanation

**We NEVER**
- Copy-paste code without explanation
- Use deprecated ROS 1 code
- Skip error handling in examples
- Write walls of text without headings
- Forget the RAG chatbot integration

## 13. Success Metrics for Hackathon Judging
- Live Docusaurus site on GitHub Pages
- Working embedded RAG chatbot (selected-text + full-book mode)
- Complete 13-week curriculum coverage
- At least 3 working ROS 2 + Isaac Sim demos
- Capstone autonomous humanoid demo video

## 14. Governance & Amendment
Any change requires new /sp.constitution command and human approval.

---
**Architecture (Supporting Files to be generated next)**
- 4 Skills → planning.md | robotics-pedagogy.md | technical-writing.md | physical-ai-mastery.md
- 4 Output Styles → module-overview.md | weekly-lesson.md | code-deep-dive.md | capstone-project.md
- 5 Sub-Agents → planner → chapter-writer → ros-code-implementer → simulation-validator → rag-chatbot-auditor

**Status:** Constitution locked ✅

## Response snapshot

{{RESPONSE_TEXT}}

## Outcome

- ✅ Impact: null
- 🧪 Tests: null
- 📁 Files: null
- 🔁 Next prompts: null
- 🧠 Reflection: null

## Evaluation notes (flywheel)

- Failure modes observed: null
- Graders run and results (PASS/FAIL): null
- Prompt variant (if applicable): null
- Next experiment (smallest change to try): null
