import os
from qdrant_client import QdrantClient, models
from qdrant_client.http.models import Distance, VectorParams
from google.generativeai import GenerativeModel
import markdown
import re

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
GEMINI_API_KEY = os.getenv("GEMINI_API_KEY") # Assuming Gemini API key is also available

client = QdrantClient(
    url=QDRANT_URL,
    api_key=QDRANT_API_KEY,
)

gemini_model = GenerativeModel("models/embedding-001") # Using an embedding model

COLLECTION_NAME = "textbook_chapters"

async def create_qdrant_collection():
    client.recreate_collection(
        collection_name=COLLECTION_NAME,
        vectors_config=VectorParams(size=768, distance=Distance.COSINE), # Adjust size based on embedding model output
    )

async def embed_text(text: str):
    response = gemini_model.embed_content(content=[text])
    # The Gemini embedding model returns a BatchEmbedContentsResponse, extract the embedding from it
    return response['embeddings'][0]['values'] # Accessing the embedding values


async def ingest_chapter_content(chapter_id: str, chapter_title: str, chapter_content: str):
    # Convert markdown to plain text for embedding
    plain_text_content = re.sub(r'<[^>]+>', '', markdown.markdown(chapter_content))

    embedding = await embed_text(plain_text_content)

    points = [
        models.PointStruct(
            id=str(chapter_id),
            vector=embedding,
            payload={"title": chapter_title, "content": chapter_content},
        )
    ]

    client.upsert(
        collection_name=COLLECTION_NAME,
        wait=True,
        points=points
    )
    return {"status": "success", "message": f"Chapter {chapter_id} ingested."}
