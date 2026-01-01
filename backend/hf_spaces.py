#!/usr/bin/env python3
"""
Hugging Face Spaces compatible FastAPI application
For deploying Physical AI & Humanoid Robotics backend
"""

import asyncio
import sys
from contextlib import asynccontextmanager
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent / "src"))

from src.main import app
from src.core.config import settings
from src.core.qdrant_client import initialize_qdrant_collections
from src.services.rag import RAGService

# Initialize RAG service
rag_service = RAGService()

@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup and shutdown events for Hugging Face Spaces"""
    # Startup
    print("Starting Physical AI Backend on Hugging Face Spaces...")
    try:
        await initialize_qdrant_collections()
        print("Qdrant collections initialized")
    except Exception as e:
        print(f"Warning: Qdrant initialization: {e}")

    # Verify Groq client
    try:
        from src.agents.chat_agent import ChatAgent
        chat_agent = ChatAgent()
        if chat_agent.groq_available:
            print("Groq client initialized")
        else:
            print("Warning: Groq client not available")
    except Exception as e:
        print(f"Warning: Chat agent initialization: {e}")

    yield

    # Shutdown
    print("Shutting down...")

# Create FastAPI app for Spaces
spaces_app = FastAPI(
    title="Physical AI Backend API",
    description="Backend API for Physical AI & Humanoid Robotics platform",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS for frontend
spaces_app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mount the main app
spaces_app.mount("/api", app)

# Health check endpoint
@spaces_app.get("/health")
async def health_check():
    return {"status": "healthy", "service": "Physical AI Backend"}

@spaces_app.get("/")
async def root():
    return {
        "service": "Physical AI & Humanoid Robotics Backend",
        "version": "1.0.0",
        "endpoints": {
            "chat": "/api/chat",
            "chat_selective": "/api/chat/selective",
            "docs": "/docs"
        }
    }

if __name__ == "__main__":
    import uvicorn
    port = int(os.environ.get("HF_SPACE_ID", "7860"))
    uvicorn.run(spaces_app, host="0.0.0.0", port=port)
