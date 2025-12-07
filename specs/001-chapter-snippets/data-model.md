# Data Model: The AI Teacher - Integrated Book & Chatbot System

**Date**: 2025-11-29
**Source**: `spec.md`

## 1.0 Core Entities

### 1.1 Project
- **name**: "The AI Teacher - Integrated Book & Chatbot System"
- **vision**: Comprehensive, AI-generated educational book about AI in education, coupled with intelligent chatbot.
- **core_deliverables**:
    - "The AI Teacher: Revolutionizing Education through Artificial Intelligence" (Book)
    - Context-aware RAG chatbot embedded within the book interface
    - Fully deployed, publicly accessible integrated system
    - Reusable AI components and comprehensive documentation
- **success_criteria**: Fully functional deployed system, Book content/structural requirements met, Chatbot query handling, Robust performance, User-friendly interface, Reusable AI patterns/components.

### 1.2 Book
- **title**: "The AI Teacher: Revolutionizing Education through Artificial Intelligence"
- **platform**: Docusaurus
- **chapter_count**: Minimum 10
- **chapter_word_count**: 1,200-1,500 words per chapter
- **chapter_subcategories**: Minimum 5 per chapter
- **content_quality_standards**: Accuracy, Depth, Accessibility, Originality (AI-generated), Citations.
- **technical_implementation**:
    - Docusaurus Configuration: Multi-language, search, responsive, dark/light, TOC, SEO, social sharing.
    - Content Organization: `docs/`, chapter folders, `index.md`, `section.md`, consistent header, breadcrumbs, nav, media, code blocks.
    - Styling and UX: Typography, visuals, interactive elements, accessibility (WCAG 2.1 AA).

### 1.3 Chapter
- **id**: (Implicit: e.g., "chapter-1")
- **title**: (e.g., "Foundations of AI in Education")
- **word_count**: 1,200-1,500
- **subcategories**: Minimum 5 sections.
- **content**: Educational text, diagrams, examples, code blocks.
- **progress_flow**: Logical progression from basic to advanced.
- **internal_consistency**: Cross-references, thematic continuity.
- **quality_standards**: Accuracy, Depth, Accessibility, Originality, Citations.

### 1.4 Chatbot System
- **primary_function**: Intelligent, context-sensitive interaction with book content.
- **query_modes**: General Knowledge, Context-Specific.
- **user_interaction_flow**: Query entry/text selection -> Mode identification -> Visual confirmation -> Processing -> Response generation -> Follow-up options.
- **response_quality_requirements**: Accuracy, Relevance, Clarity, Citation, Handling Uncertainty.

### 1.5 Query
- **type**: `enum("General Knowledge", "Context-Specific")`
- **question**: `string` (User's input question)
- **context**: `string` (User-selected text, for context-specific queries)
- **conversation_id**: `string` (optional)

### 1.6 ChatResponse
- **answer**: `string`
- **sources**: `list` of `string` (references to book sections)
- **confidence**: `float`

## 2.0 Technical Architecture Components

### 2.1 Backend System (FastAPI)
- **endpoints**:
    - `POST /chat/general`: General knowledge queries
    - `POST /chat/context`: Context-specific queries
    - `POST /documents/ingest`: Book content processing
    - `GET /health`: System status check
- **data_models**:
    - `QueryRequest`: `{question: str, conversation_id: str?}`
    - `ContextQueryRequest`: `{question: str, context: str, conversation_id: str?}`
    - `ChatResponse`: `{answer: str, sources: list, confidence: float}`

### 2.2 Vector Database (Qdrant Cloud)
- **collection_ai_teacher_book**:
    - `vectors`: 768-dimension (or model-specific)
    - `payload`:
        - `chapter_id`: `string`
        - `section_id`: `string`
        - `content`: `string`
        - `word_count`: `integer`
        - `metadata`: `object`
- **collection_temporary_context**:
    - `vectors`: Same dimension as main collection
    - `payload`:
        - `session_id`: `string`
        - `timestamp`: `datetime`
        - `content`: `string`

### 2.3 Processing Pipeline
- **content_ingestion**: Book -> Text Chunking -> Embedding Generation -> Qdrant Storage
- **query_processing**: User Input -> Intent Classification -> Vector Search -> Response Generation
- **context_handling**: Text Selection -> Temporary Embedding -> Isolated Search -> Response

### 2.4 Frontend Integration
- **components**: Embedded Chatbot Widget, Text Selection Interface.
- **communication_functions**:
    - `async function askGeneralQuestion(question)`
    - `async function askContextQuestion(question, selectedText)`
    - `function captureTextSelection()`

## 3.0 AI Development Components

### 3.1 Gemini CLI Subagent
- **types**:
    - Content Generation Agent (Chapter writing, section expansion, consistency, quality validation)
    - Code Development Agent (FastAPI endpoint, vector database, frontend integration, testing)
    - Deployment Agent (IaC, CI/CD, monitoring/logging, rollback)
- **skills**:
    - Text Chunking Skill
    - Embedding Management Skill
    - Response Generation Skill
    - Code Review/Improvement Skill
    - Testing Generation/Execution Skill
    - Documentation Generation Skill
    - Performance Optimization Skill
- **prompt_engineering_standards**: Consistent Templates, Context Management, Error Handling, Quality Validation.

### 3.2 Development Workflow
- **phases**: Specification Interpretation, Iterative Development, Integration & Testing.
- **quality_assurance**: Automated Testing, Integration Testing, Performance Testing, User Acceptance Testing.

## 4.0 Operational and Evaluation Entities

### 4.1 Deployment
- **hosting_platform_options**: GitHub Pages, Vercel, Netlify (Docusaurus); Railway, Render, Heroku (FastAPI).
- **service_dependencies**: Qdrant Cloud, OpenAI API, Monitoring service (optional).
- **architecture**: Frontend (build command, output dir, env vars), Backend (platform, runtime, env vars).
- **continuous_deployment**: Build Pipeline (commit, testing, build, deploy staging, deploy prod), Rollback Strategy.

### 4.2 Evaluation Criteria
- **metrics**:
    - Technical Implementation (Book Quality, Chatbot Functionality, System Architecture)
    - AI Development Excellence (Gemini CLI Utilization, Spec-Driven Development)
    - Deployment & Operational Excellence (Live System, Documentation & Maintenance)
    - Bonus Points (Advanced Features, Technical Excellence)

### 4.3 Milestone
- **name**: Basic Book Structure, Core Chatbot Functionality, Feature Completion, Production Ready.
- **quality_gates**: Specific criteria for each milestone.

### 4.4 Technical Risk
- **type**: API Rate Limiting, Content Quality, Integration Complexity, Deployment Issues.
- **mitigation_strategies**: Fallback Content, Quality Validation, Modular Development, Multi-Platform Testing.

### 4.5 Submission Requirement
- **type**: Source Code Repository, Live Deployment URL, Project Documentation, Demo Video.
- **repository_structure**: `project-repo/`, `book/`, `chatbot/`, `frontend/`, `agents/`, `docs/`, `deployment/`, `README.md`.
- **documentation_requirements**: Technical (Setup, Architecture, API, Deployment), User (Book Navigation, Chatbot Usage, Troubleshooting).
