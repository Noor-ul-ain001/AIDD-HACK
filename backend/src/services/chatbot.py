import os
import uuid
from typing import List, Dict, Optional
import google.generativeai as genai
from qdrant_client import QdrantClient
from qdrant_client.http.models import Filter, FieldCondition, Range
from backend.src.services.ingestion import embed_text, COLLECTION_NAME

# Configure Gemini API
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

# Initialize Qdrant client
QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")

qdrant_client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

async def retrieve_context_from_qdrant(query_embedding: List[float], limit: int = 3) -> List[Dict]:
    search_result = qdrant_client.search(
        collection_name=COLLECTION_NAME,
        query_vector=query_embedding,
        limit=limit
    )
    context = []
    for hit in search_result:
        context.append(hit.payload)
    return context

async def generate_gemini_response(prompt: str, context: Optional[List[Dict]] = None) -> str:
    model = genai.GenerativeModel('gemini-1.5-pro')
    
    full_prompt = prompt
    if context:
        context_str = "\n".join([f"Title: {c.get('title', '')}\nContent: {c.get('content', '')}" for c in context])
        full_prompt = f"Given the following context from the textbook:\n\n{context_str}\n\nAnswer the following question:\n{prompt}"

    response = await model.generate_content_async(full_prompt)
    return response.text

async def get_chatbot_response(query: str, session_id: str) -> str:
    query_embedding = await embed_text(query)
    context = await retrieve_context_from_qdrant(query_embedding)
    response = await generate_gemini_response(query, context)
    return response

async def get_selected_text_chatbot_response(selected_text: str, query: str, session_id: str) -> str:
    context = [{"content": selected_text}] # Treat selected text as direct context
    response = await generate_gemini_response(query, context)
    return response
