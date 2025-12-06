project:
  type: ai-native-textbook
  hackathon: Panaversity Physical AI & Humanoid Robotics Textbook Hackathon (November 2025)
  title: "Physical AI & Humanoid Robotics: Bridging Digital Intelligence with Embodied Agents"
  short_title: "Physical AI Textbook"
  author: "Ramesha Javed"
  author_credentials: "GIAIC Quarter 4 • Governor Sindh Initiative for AI • Future Panaversity Core Team Candidate"
  version: "1.0.0"
  license: "CC-BY-SA-4.0"
  primary_language: "English"
  secondary_language: "Urdu (full real-time translation)"
  cover_image: "/assets/cover-physical-ai.jpg"
  custom_domain: "https://physical-ai.panaversity.org"
  github_repo: "https://github.com/ramesha-javed/physical-ai-textbook-ramesha"

target_score:
  base: 100/100
  bonus_target: 200/200
  total_goal: 300 points

core_deliverables:
  - Fully AI-generated textbook using only Spec-Kit Plus + Claude Code (zero manual writing)
  - 14 complete chapters exactly matching Panaversity course outline
  - Deployed on GitHub Pages with 100% custom professional UI (NO default Docusaurus theme)
  - Embedded RAG chatbot on every page answering from whole book + selected text only

bonus_features_all_implemented:
  - authentication:
      tool: "Better-Auth (latest)"
      signup_survey: true
      questions:
        - "Python proficiency level"
        - "ROS 2 experience"
        - "Access to GPU / robotics hardware"
        - "Current education / career stage"
      storage: "Neon Serverless Postgres"
  - personalization:
      button: "Adapt to My Level"
      levels: ["Beginner", "Intermediate", "Advanced"]
      dynamic_rewriting: true
  - urdu_translation:
      button: "یہ باب اردو میں پڑھیں"
      model: "GPT-4o with Urdu fine-tuning prompts"
      typography: "Jameel Noori Nastaleeq + Noto Nastaliq"
      caching: true
  - claude_subagents:
      count: 12 reusable subagents
      purpose: chapter writing, diagrams, code, Urdu, personalization, indexing


module_1_ros2:
  chapters:
    - 01-ros2-introduction.md:
        title: "ROS 2 – The Robotic Nervous System"
        user_stories:
          - US1: As a beginner, I want to understand what ROS 2 is and its core concepts so I can grasp its importance in robotics.
          - US2: As a beginner, I want to see simple `rclpy` code examples for basic ROS 2 operations with Roman Urdu comments so I can understand the syntax and logic.
          - US3: As a learner, I want to test my understanding with MCQs on ROS 2 introduction to reinforce my knowledge.
    - 02-nodes-topics-services.md:
        title: "ROS 2 Nodes, Topics, and Services"
        user_stories:
          - US1: As a learner, I want to understand ROS 2 nodes, topics, and services so I can build modular robot applications.
          - US2: As a learner, I want to see `rclpy` code examples for creating and communicating between nodes using topics and services, with Roman Urdu comments, to apply these concepts.
          - US3: As a learner, I want to test my knowledge with MCQs on nodes, topics, and services.
    - 03-building-ros-packages.md:
        title: "Building Your First ROS 2 Package in Python"
        user_stories:
          - US1: As a new developer, I want to learn how to create and build a ROS 2 Python package so I can organize my robotic projects.
          - US2: As a new developer, I want to see step-by-step `rclpy` code examples for package creation, compilation, and execution, with Roman Urdu comments, to follow along.
          - US3: As a learner, I want to test my understanding with MCQs on building ROS 2 packages.
    - 04-urdf-xacro-humanoid.md:
        title: "URDF & Robot Description Mastery"
        user_stories:
          - US1: As a robot designer, I want to understand URDF and Xacro to accurately describe robot kinematics and visuals.
          - US2: As a robot designer, I want to see URDF/Xacro examples for defining a humanoid robot, with Roman Urdu comments in the conceptual sections, to guide my own designs.
          - US3: As a learner, I want to test my knowledge with MCQs on URDF and Xacro.
    - 05-launch-and-parameters.md:
        title: "ROS 2 Launch and Parameters"
        user_stories:
          - US1: As a robot operator, I want to learn about ROS 2 launch files and parameters to easily start and configure multiple robot nodes.
          - US2: As a robot operator, I want to see `rclpy` launch file examples for starting multiple nodes and passing parameters, with Roman Urdu comments, to automate my robot's startup.
          - US3: As a learner, I want to test my understanding with MCQs on ROS 2 launch files and parameters.

book_structure:

  total_chapters: 14
  chapters:
    1. Introduction to Physical AI and Embodied Intelligence
    2. Why Humanoid Robots Will Redefine the Future of Work
    3. ROS 2 – The Robotic Nervous System
    4. Building Your First ROS 2 Package in Python
    5. URDF & Robot Description Mastery
    6. Gazebo – Creating Digital Twins
    7. Unity for High-Fidelity Human-Robot Interaction
    8. NVIDIA Isaac Sim – Next-Gen Robot Training Platform
    9. Vision-Language-Action (VLA) Models
    10. Voice to Physical Action (Whisper + GPT-4o + ROS 2)
    11. Bipedal Locomotion & Balance Control
    12. Dexterous Manipulation with Humanoid Hands
    13. Capstone: Autonomous Conversational Humanoid Robot
    14. Career Paths in Physical AI – Pakistan & Global Landscape

ui_ux_requirements:
  remove_docusaurus_default_theme: true
  design_inspiration: "Apple.com + Perplexity.ai + Linear.app + Arc Browser"
  style: "Premium glassmorphism + subtle neumorphism"
  color_palette: "Indigo-600 primary + neutral glass cards"
  typography:
    english: "Inter, system-ui"
    urdu: "Jameel Noori Nastaleeq, Noto Nastaliq Urdu"
  must_have_components:
    - Custom animated navbar with progress ring
    - Floating draggable RAG chatbot (bottom-right)
    - Chapter header with two buttons (Personalize + Urdu)
    - Dark/light mode with system sync
    - Mobile-perfect responsive (320px+)
    - Reading time + difficulty badge
    - Mermaid diagrams + interactive code blocks

technical_stack:
  frontend: "Docusaurus core + Custom React 19 + Tailwind CSS + Framer Motion"
  backend: "FastAPI"
  auth: "Better-Auth"
  database: "Neon Serverless Postgres"
  vector_store: "Qdrant Cloud Free Tier"
  llm_layer: "OpenAI Agents SDK + Groq fallback"
  content_generation: "Claude Code + Spec-Kit Plus"
  deployment: "GitHub Pages + GitHub Actions CI/CD"

success_criteria:
  - 100% Spec-Kit Plus compliant
  - Zero manual content or code writing
  - All 200 bonus points features fully working
  - Production-ready within hackathon deadline
  - Judges can sign up → get personalized content → switch to Urdu → ask RAG anything → be amazed

submission_ready:
  constitution_md: true
  spec_book_yaml: true
  demo_video: "<90 seconds, recorded via NotebookLM + screen"
  live_presentation: "Ready for 30 Nov 2025 6PM Zoom"

dedication: |
  Dedicated to Zia Khan Sir, Rehan Allahwala Sir, Junaid, Wania,
  and the entire Panaversity vision of making Pakistan the global leader in AI & Robotics education.

generated_by: "Ramesha Javed × Grok 4 (Official Hackathon Partner)"
status: "READY TO WIN – InshaAllah we will be selected for Panaversity Core Team 🇵🇰🏆