import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
import os

# Adjust the import path for the main FastAPI app
# Assuming the test is run from the project root or 'backend' directory
from backend.src.api.main import app 

client = TestClient(app)

@pytest.fixture(autouse=True)
def mock_dependencies():
    """
    Mocks external dependencies for ingestion tests, including QdrantClient.
    """
    # Mock environment variables
    with patch.dict(os.environ, {
        "QDRANT_URL": "http://mock-qdrant-url",
        "QDRANT_API_KEY": "mock-qdrant-key",
        "GEMINI_API_KEY": "mock-gemini-key"
    }), \
         patch('backend.src.services.ingestion.client') as mock_ingestion_services_qdrant_client_instance, \
         patch('backend.src.services.chatbot.qdrant_client') as mock_chatbot_qdrant_client_instance, \
         patch('backend.src.services.ingestion.embed_text', new_callable=AsyncMock) as mock_embed_text, \
         patch('backend.src.services.ingestion.gemini_model') as mock_ingestion_gemini_model, \
         patch('backend.src.api.ingestion.create_qdrant_collection', new_callable=AsyncMock) as mock_api_create_qdrant_collection, \
         patch('backend.src.api.ingestion.ingest_chapter_content', new_callable=AsyncMock) as mock_api_ingest_chapter_content, \
         patch('backend.src.api.ingestion.get_all_md_files', new_callable=AsyncMock) as mock_api_get_all_md_files:
        
        # Configure mock_embed_text to return a dummy embedding
        mock_embed_text.return_value = [0.1] * 768 # Dummy embedding

        # Configure mock_ingestion_gemini_model.embed_content to return a dummy response
        mock_ingestion_gemini_model.embed_content.return_value = {
            'embeddings': [{'values': [0.1] * 768}]
        }


        yield mock_api_create_qdrant_collection, mock_api_ingest_chapter_content, mock_embed_text, mock_ingestion_services_qdrant_client_instance, mock_chatbot_qdrant_client_instance, mock_ingestion_gemini_model.embed_content, mock_api_get_all_md_files

@pytest.fixture
def mock_os_walk():
    """
    Mocks os.walk to simulate a directory structure with markdown files.
    """
    with patch('os.walk') as mock_walk:
        yield mock_walk

@pytest.fixture
def mock_open():
    """
    Mocks the built-in open function to simulate reading file content.
    """
    with patch('builtins.open', MagicMock()) as mock_file_open:
        yield mock_file_open

def create_mock_md_file(filename, content):
    """Helper to create a mock markdown file for os.walk and open."""
    mock_file = MagicMock()
    mock_file.read.return_value = content
    mock_file.__enter__.return_value = mock_file
    mock_file.__exit__.return_value = None
    return mock_file

@pytest.mark.asyncio
async def test_ingest_docs_success(mock_dependencies, mock_os_walk, mock_open):
    mock_api_create_qdrant_collection, mock_api_ingest_chapter_content, mock_embed_text, mock_ingestion_services_qdrant_client_instance, mock_chatbot_qdrant_client_instance, mock_ingestion_gemini_embed_content, mock_api_get_all_md_files = mock_dependencies

    # Configure mock_api_get_all_md_files
    mock_api_get_all_md_files.return_value = [
        'C:/Users/user/Desktop/noor/frontend/docs/module-1/chapter1.md',
        'C:/Users/user/Desktop/noor/frontend/docs/module-1/chapter2.mdx',
        'C:/Users/user/Desktop/noor/frontend/docs/module-2/chapter3.md',
    ]

    # Setup mock open to return content for each file
    mock_open.side_effect = [
        create_mock_md_file('chapter1.md', '# Chapter 1 Title\nContent of chapter 1.'),
        create_mock_md_file('chapter2.mdx', '## Chapter 2 Title\nContent of chapter 2.'),
        create_mock_md_file('chapter3.md', 'Content of chapter 3 without title.'),
    ]

    response = client.post("/ingestion/ingest")

    assert response.status_code == 201
    assert response.json() == {"status": "success", "message": "Ingested 3 documents."}

    mock_api_create_qdrant_collection.assert_called_once()
    assert mock_api_ingest_chapter_content.call_count == 3

    # Verify calls to ingest_chapter_content
    # Call 1: chapter1.md
    call_args_1 = mock_api_ingest_chapter_content.call_args_list[0].kwargs
    assert call_args_1['chapter_title'] == 'Chapter 1 Title'
    assert call_args_1['chapter_content'] == '# Chapter 1 Title\nContent of chapter 1.'

    # Call 2: chapter2.mdx
    call_args_2 = mock_api_ingest_chapter_content.call_args_list[1].kwargs
    assert call_args_2['chapter_title'] == 'Chapter 2 Title'
    assert call_args_2['chapter_content'] == '## Chapter 2 Title\nContent of chapter 2.'

    # Call 3: chapter3.md
    call_args_3 = mock_api_ingest_chapter_content.call_args_list[2].kwargs
    assert call_args_3['chapter_title'] == 'chapter3.md' # Uses filename as title if no # or ##
    assert call_args_3['chapter_content'] == 'Content of chapter 3 without title.'

@pytest.mark.asyncio
async def test_ingest_docs_no_files_found(mock_dependencies, mock_os_walk):
    mock_api_create_qdrant_collection, mock_api_ingest_chapter_content, mock_embed_text, mock_ingestion_services_qdrant_client_instance, mock_chatbot_qdrant_client_instance, mock_ingestion_gemini_embed_content, mock_api_get_all_md_files = mock_dependencies
    
    # Configure mock_api_get_all_md_files to return no files
    mock_api_get_all_md_files.return_value = []

    response = client.post("/ingestion/ingest")

    assert response.status_code == 404
    assert response.json() == {"detail": "No markdown files found in frontend/docs."}
    mock_api_create_qdrant_collection.assert_called_once()
    mock_api_ingest_chapter_content.assert_not_called()

@pytest.mark.asyncio
async def test_ingest_docs_internal_error(mock_dependencies, mock_os_walk):
    mock_api_create_qdrant_collection, mock_api_ingest_chapter_content, mock_embed_text, mock_ingestion_services_qdrant_client_instance, mock_chatbot_qdrant_client_instance, mock_ingestion_gemini_embed_content, mock_api_get_all_md_files = mock_dependencies
    
    # Configure mock_api_get_all_md_files to return some files so it doesn't raise 404
    mock_api_get_all_md_files.return_value = ['dummy.md']
    
    mock_api_create_qdrant_collection.side_effect = Exception("Qdrant connection error")

    response = client.post("/ingestion/ingest")

    assert response.status_code == 500
    assert response.json() == {"detail": "Qdrant connection error"}
    mock_api_create_qdrant_collection.assert_called_once()
