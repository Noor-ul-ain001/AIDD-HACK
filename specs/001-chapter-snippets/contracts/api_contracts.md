# API Contracts: The AI Teacher - Integrated Book & Chatbot System

**Date**: 2025-11-29
**Source**: `spec.md`

## 1.0 Overview

This document outlines the API endpoints and data models for the Chatbot Backend System, implemented using FastAPI, as specified in the project specification.

## 2.0 Endpoints

### 2.1 `POST /chat/general`
- **Description**: Handles general knowledge queries against the entire book corpus.
- **Request Body**: `QueryRequest`
- **Response Body**: `ChatResponse`
- **Functionality**: Performs semantic search across all chapters and generates a comprehensive response.

### 2.2 `POST /chat/context`
- **Description**: Processes context-specific queries based on user-selected text passages.
- **Request Body**: `ContextQueryRequest`
- **Response Body**: `ChatResponse`
- **Functionality**: Answers questions limited to the provided context, utilizing temporary vector storage for selected text.

### 2.3 `POST /documents/ingest`
- **Description**: Endpoint for ingesting and processing book content into the vector database.
- **Request Body**: (To be defined, likely involves content, chapter_id, etc.)
- **Response Body**: (To be defined, e.g., status of ingestion)
- **Functionality**: Triggers the text chunking, embedding generation, and storage pipeline for book content.

### 2.4 `GET /health`
- **Description**: Provides a simple health check to monitor the system's status.
- **Request**: No request body.
- **Response**: `JSON` (e.g., `{"status": "healthy"}`)
- **Functionality**: Indicates the operational status of the backend service.

## 3.0 Data Models

### 3.1 `QueryRequest`
- **Description**: Request model for general knowledge queries.
- **Properties**:
    - `question`: `string` (The user's query)
    - `conversation_id`: `string` (Optional, for session management)

### 3.2 `ContextQueryRequest`
- **Description**: Request model for context-specific queries.
- **Properties**:
    - `question`: `string` (The user's query)
    - `context`: `string` (The user-selected text)
    - `conversation_id`: `string` (Optional, for session management)

### 3.3 `ChatResponse`
- **Description**: Response model for chatbot queries.
- **Properties**:
    - `answer`: `string` (The generated answer)
    - `sources`: `list<string>` (References or citations from the book)
    - `confidence`: `float` (Confidence score of the answer)
