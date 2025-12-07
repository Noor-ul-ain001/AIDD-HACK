# Tasks for RAG Chatbot Implementation

## Setup Phase

- [X] Task: Initialize Qdrant collection on startup or first ingestion.
    - Files: `backend/src/services/ingestion.py`
    - Notes: Ensure the `create_qdrant_collection` function is called at an appropriate time.

## Ingestion Phase

- [X] Task: Implement document ingestion API endpoint in `backend/src/api/ingestion.py`.
    - Files: `backend/src/api/ingestion.py`
    - Notes: This includes scanning `frontend/docs` for markdown/mdx files, reading their content, and preparing them for ingestion. (Already implemented in `ingestion.py`)
- [X] Task: Update `backend/src/api/main.py` to include the `ingestion_router`.
    - Files: `backend/src/api/main.py`
    - Notes: Ensure the new `/ingestion` endpoint is accessible. (Already implemented)
- [X] Task: Implement file content reading and processing logic.
    - Files: `backend/src/api/ingestion.py`
    - Notes: The `ingest_docs` function needs to correctly read markdown files and extract title/content.
- [X] Task: Integrate `ingest_chapter_content` call for each processed file.
    - Files: `backend/src/api/ingestion.py`
    - Notes: Ensure unique `chapter_id` generation and correct passing of `chapter_title` and `chapter_content`.

## Chatbot Integration Phase

- [X] Task: Verify `backend/src/services/chatbot.py` uses Qdrant for context retrieval.
    - Files: `backend/src/services/chatbot.py`
    - Notes: Confirm the existing `get_chatbot_response` and `get_selected_text_chatbot_response` functions correctly query Qdrant.

## Testing Phase

- [X] Task: Write unit tests for `backend/src/api/ingestion.py`.
    - Files: `backend/tests/test_ingestion.py` (create if not exists)
    - Notes: Test endpoint behavior, file scanning, content reading, and `ingest_chapter_content` calls.
- [X] Task: Write integration tests for ingestion and chat flow.
    - Files: `backend/tests/test_integration.py` (create if not exists)
    - Notes: Test the full pipeline from document upload/ingestion to chatbot response.

## Polish Phase

- [X] Task: Add a UI element in the frontend to trigger document ingestion.
    - Files: `frontend/src/pages/dashboard.tsx` or similar.
    - Notes: This could be an admin button to re-ingest all documents.
