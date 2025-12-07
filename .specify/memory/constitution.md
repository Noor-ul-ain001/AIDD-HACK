<!--
Sync Impact Report:
- Version change: 0.0.0 → 1.0.0
- List of modified principles: Initial constitution, all principles are new.
- Added sections: All sections are new.
- Removed sections: All sections in the old template were replaced.
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md
  - ✅ .specify/templates/spec-template.md
  - ✅ .specify/templates/tasks-template.md
- Follow-up TODOs:
  - TODO(RATIFICATION_DATE): Confirm if 2025-12-02 is the correct ratification date.
-->
# **"Physical AI & Humanoid Robotics" Textbook Project** Constitution

## Core Mission
Create an AI-native technical textbook for "Physical AI & Humanoid Robotics" using Docusaurus with integrated RAG chatbot, authentication, personalization, and translation features. All content generation will be done using **Gemini API**.

## Core Principles

### I. Foundation Requirements
Tech Stack:
- Docusaurus v3.x (with TypeScript)
- Spec-Kit Plus template for AI-native content generation
- GitHub Pages for deployment
- Python FastAPI backend for RAG
- Neon PostgreSQL for vector storage
- Qdrant Cloud for embeddings
- OpenAI API for RAG generation (or Gemini if preferred)
- Better Auth for authentication
- Gemini API for ALL content generation (replace all Claude references)

### II. Book Structure & First Page Customization
**Custom Docusaurus First Page MUST include:**
1. **Hero Section:**
   - Course title: "Physical AI & Humanoid Robotics"
   - Subtitle: "Bridging the Digital Brain and Physical Body"
   - Interactive 3D model placeholder (using Three.js integration)
   - Live demo video embed of humanoid robot

2. **Course Overview Cards:**
   - Module 1: Robotic Nervous System (ROS 2)
   - Module 2: Digital Twin (Gazebo & Unity)
   - Module 3: AI-Robot Brain (NVIDIA Isaac)
   - Module 4: Vision-Language-Action (VLA)

3. **Hardware Visualization:**
   - Interactive comparison table: Workstation vs Edge Kit vs Robot Lab
   - 3D models of: NVIDIA Jetson, RealSense Camera, Unitree robots
   - Cost calculator for lab setup

### III. Content Generation Rules
**Using Gemini API/Spec-Kit Plus:**
1. **Chapter Structure per Module:**
   ```
   Each Chapter = {
     Learning Objectives: [list]
     Theory: {concepts, equations, diagrams}
     Practical: {code examples, ROS2 nodes, simulation steps}
     Hardware Integration: {setup guides, wiring diagrams}
     AI Agents: {Gemini prompts and configurations for each topic}
   }
   ```
2. **Required Content Depth:**
   - ROS 2: Complete rclpy implementation examples
   - Gazebo: URDF/SDF files with physics parameters
   - NVIDIA Isaac: Python scripts for perception pipelines
   - VLA: Complete OpenAI Whisper + GPT-4 integration code
3. **Interactive Elements:**
   - Live code editors for ROS 2 examples
   - Gazebo simulation embeds (WebGL)
   - Jupyter notebook integration for AI training

### IV. RAG Chatbot Implementation
**System Architecture:**
```python
Components:
1. FastAPI backend with:
   - /chat endpoint (handles book queries)
   - /selected-chat endpoint (handles selected text queries)
   - /ingest endpoint (processes book content)
2. Vector Database Schema:
   - chapters (id, title, content, embeddings)
   - user_sessions (id, query_history, personalization_prefs)
   - selected_text_cache (session_id, text, embeddings)
3. Qdrant Collections:
   - main_book_content (1024-dim embeddings)
   - user_context (personalized embeddings)
```
**Chatbot Features MUST include:**
1. Context-aware responses based on chapter content
2. Selected text analysis (highlight → query)
3. Code explanation with execution steps
4. Hardware troubleshooting assistant

### V. Authentication & Personalization
**Better Auth Integration:**
1. **Signup Flow:**
   ```
   Required fields: {
     email,
     password,
     software_background: [Beginner, Intermediate, Advanced],
     hardware_experience: [None, Arduino/RPi, ROS, Robotics Kit],
     preferred_learning: [Visual, Code-heavy, Theory, Hands-on]
   }
   ```
2. **Personalization Engine:**
   ```javascript
   // Chapter personalization button
   personalizeChapter(chapterId, userProfile) {
     adjustContent({
       code_examples: userProfile.software_background,
       hardware_details: userProfile.hardware_experience,
       theory_depth: userProfile.preferred_learning
     })
   }
   ```
3. **User Dashboard:**
   - Progress tracking per module
   - Recommended labs based on background
   - Hardware requirement calculator

### VI. Urdu Translation System
**Implementation Rules:**
1. **Translation Button per Chapter:**
   ```javascript
   translateToUrdu(chapterContent) {
     // Use Gemini API for context-aware translation
     // Technical terms: Keep English with Urdu explanation
     // Code blocks: Preserve original, translate comments
   }
   ```
2. **Bilingual Display:**
   - Side-by-side English/Urdu view option
   - Technical glossary: English term → Urdu definition
   - Voice narration in Urdu for theory sections

### VII. Gemini Integration Strategy
**Gemini API Configuration:**
1. **Content Generation Prompts:**
   - Use `gemini-1.5-pro` for technical content
   - Use `gemini-1.5-flash` for translations and simpler tasks
   - Implement temperature control: 0.3 for technical accuracy, 0.7 for creative explanations
2. **Specialized Prompts for Each Module:**
   ```
   Module 1 Prompt: "Generate ROS 2 node for [topic] with error handling"
   Module 2 Prompt: "Create Gazebo world file for humanoid robot training"
   Module 3 Prompt: "Write NVIDIA Isaac perception pipeline code"
   Module 4 Prompt: "Design VLA system architecture with code integration"
   ```
3. **Content Validation System:**
   - Secondary Gemini call to validate technical accuracy
   - Code syntax checking with AST parsing
   - Hardware specification verification

### VIII. Deployment Specifications
**GitHub Repository Structure:**
```
/textbook
  /docs
    /module-1
      /chapters
      /code-examples
      /simulations
    /module-2
    /module-3
    /module-4
  /src
    /components
      Chatbot.tsx
      PersonalizationButton.tsx
      TranslationButton.tsx
    /pages
      index.tsx  # Custom first page
  /backend
    rag_api/
    auth/
    gemini_integration/
  /public
    3d-models/
    hardware-images/
```
**Deployment Pipeline:**
1. Docusaurus build → GitHub Pages
2. FastAPI backend → Vercel/Heroku
3. Database: Neon PostgreSQL + Qdrant Cloud

### IX. Gemini-Specific Content Rules
1. **Technical Content:**
   - All code must be tested and executable
   - Include ROS 2 Humble/Iron version compatibility notes
   - Add troubleshooting sections for common hardware issues
2. **Hardware Guides:**
   - Step-by-step setup with photos/diagrams
   - Configuration files for each component
   - Safety protocols and best practices
3. **AI Integration:**
   - Complete API integration examples
   - Error handling for network issues
   - Model optimization tips for edge deployment

### X. Content Validation Rules
- All code examples must be executable
- Hardware specifications must be accurate and updated
- Simulation steps must be reproducible
- AI integration examples must use current APIs
- Safety protocols for physical robotics must be emphasized

## Governance

### Execution Priority
1. Set up Docusaurus with Spec-Kit Plus
2. Create custom first page with interactive elements
3. Generate Module 1 content using Gemini API
4. Implement RAG chatbot backend
5. Integrate Better Auth with background questions
6. Add personalization and translation features using Gemini
7. Deploy complete system
8. Create demo video

### Technical Constraints
- Use TypeScript for frontend
- Python 3.10+ for backend
- OpenAI/Gemini for RAG generation
- Gemini API for ALL content generation
- All external APIs must have fallback mechanisms
- Mobile-responsive design required

**Version**: 1.0.0 | **Ratified**: 2025-12-02 | **Last Amended**: 2025-12-02