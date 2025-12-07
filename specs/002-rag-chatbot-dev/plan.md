# Plan for RAG Chatbot

## Objective
Implement a Retrieval-Augmented Generation (RAG) chatbot that can ingest markdown documents from the `frontend/docs` directory and respond to user queries based on the ingested content.

## Tech Stack
- **Backend:** FastAPI (Python)
- **Vector Database:** Qdrant (via `backend/src/services/ingestion.py`)
- **Frontend:** React (via Docusaurus)

## Architecture Overview
The system consists of a FastAPI backend and a React frontend. The backend exposes API endpoints for chat interactions and document ingestion. The frontend provides a chatbot UI that interacts with these backend endpoints.

### Key Components:
- **FastAPI Application (`backend/src/api/main.py`):** Main entry point, handles routing and middleware (e.g., CORS).
- **Chat API (`backend/src/api/chat.py`):** Handles user chat queries and routes them to the chatbot service.
- **Ingestion API (`backend/src/api/ingestion.py`):** (Newly added) Handles the ingestion of documents into the Qdrant vector database.
- **Chatbot Service (`backend/src/services/chatbot.py`):** Contains the core logic for retrieving context from Qdrant and generating responses using an LLM.
- **Ingestion Service (`backend/src/services/ingestion.py`):** Contains the logic for processing documents and storing them in Qdrant.
- **Frontend Chatbot Component (`frontend/src/components/Chatbot.tsx`):** Provides the user interface for chat interactions.

## High-Level Plan
1.  **Backend Ingestion Endpoint:** Implement an endpoint (`/ingestion/ingest`) that scans `frontend/docs` for markdown files, processes them, and stores their content in Qdrant.
2.  **Frontend Ingestion Trigger (Optional for now):** (Could be added later) A UI element to trigger the ingestion process.
3.  **Chatbot Enhancement:** Ensure the chatbot service correctly uses the ingested data from Qdrant to provide relevant responses.
4.  **Error Handling and Robustness:** Improve error messages and ensure stability.

## File Structure (Relevant sections)
- `backend/src/api/main.py`: FastAPI app setup, router inclusion, CORS.
- `backend/src/api/chat.py`: Chat endpoints (`/chat`, `/selected-chat`).
- `backend/src/api/ingestion.py`: New ingestion endpoint (`/ingestion/ingest`).
- `backend/src/services/chatbot.py`: Core chatbot logic.
- `backend/src/services/ingestion.py`: Document processing and Qdrant interaction.
- `frontend/src/components/Chatbot.tsx`: Frontend UI for the chatbot.
- `frontend/docs/`: Directory containing markdown files to be ingested.
