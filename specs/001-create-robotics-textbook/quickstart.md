# Quickstart: AI-Native Textbook for Physical AI & Humanoid Robotics

This guide provides a quick overview of how to set up and run the project locally.

## Prerequisites

- Node.js (v18 or higher)
- Python (v3.10 or higher)
- Docker

## Frontend Setup

1. Navigate to the `frontend` directory:
   ```bash
   cd frontend
   ```
2. Install the dependencies:
   ```bash
   npm install
   ```
3. Start the development server:
   ```bash
   npm start
   ```
The Docusaurus site will be available at `http://localhost:3000`.

## Backend Setup

1. Navigate to the `backend` directory:
   ```bash
   cd backend
   ```
2. Create a virtual environment:
   ```bash
   python -m venv venv
   ```
3. Activate the virtual environment:
   - **Windows**: `venv\Scripts\activate`
   - **macOS/Linux**: `source venv/bin/activate`
4. Install the dependencies:
   ```bash
   pip install -r requirements.txt
   ```
5. Start the development server:
   ```bash
   uvicorn main:app --reload
   ```
The FastAPI server will be available at `http://localhost:8000`.

## Database Setup

This project uses Neon for PostgreSQL and Qdrant for vector storage. You will need to create accounts on their respective cloud platforms and obtain the necessary credentials.

1. **Neon**: Create a new project and get the connection string.
2. **Qdrant**: Create a new cluster and get the API key and URL.

Set the following environment variables in a `.env` file in the `backend` directory:

```
DATABASE_URL=your_neon_connection_string
QDRANT_URL=your_qdrant_url
QDRANT_API_KEY=your_qdrant_api_key
```
