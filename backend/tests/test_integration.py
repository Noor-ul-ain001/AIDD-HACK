import pytest
from fastapi.testclient import TestClient
from unittest.mock import AsyncMock, patch, MagicMock
import os

from backend.src.api.main import app 

client = TestClient(app) 

@pytest.fixture(autouse=True)
def mock_integration_dependencies():
    """
    Mocks external dependencies for integration tests.
    """
    with patch.dict(os.environ, {
        "QDRANT_URL": "http://mock-qdrant-url",
        "QDRANT_API_KEY": "mock-qdrant-key",
        "GEMINI_API_KEY": "mock-gemini-key"
    }), \
         patch('backend.src.services.ingestion.client') as mock_ingestion_qdrant_client_instance, \
         patch('backend.src.services.chatbot.qdrant_client') as mock_chatbot_qdrant_client_instance, \
         patch('backend.src.services.ingestion.embed_text', new_callable=AsyncMock) as mock_ingestion_embed_text, \
         patch('backend.src.services.chatbot.embed_text', new_callable=AsyncMock) as mock_chatbot_embed_text, \
         patch('backend.src.services.chatbot.generate_gemini_response', new_callable=AsyncMock) as mock_generate_gemini_response, \
         patch('backend.src.api.ingestion.get_all_md_files', new_callable=AsyncMock) as mock_api_get_all_md_files:
        
        # Configure mocks for ingestion Qdrant client
        mock_ingestion_qdrant_client_instance.recreate_collection = MagicMock()
        mock_ingestion_qdrant_client_instance.upsert = MagicMock()

        # Configure mocks for chatbot Qdrant client
        mock_chatbot_qdrant_client_instance.search = MagicMock(return_value=[
            MagicMock(payload={"title": "Context Title", "content": "Context Content"})
        ])

        # Configure embed_text mocks
        mock_ingestion_embed_text.return_value = [0.1] * 768
        mock_chatbot_embed_text.return_value = [0.2] * 768

        # Configure Gemini response mock
        mock_generate_gemini_response.return_value = "Mocked Gemini response."

        yield mock_ingestion_qdrant_client_instance, mock_chatbot_qdrant_client_instance, \
              mock_ingestion_embed_text, mock_chatbot_embed_text, \
              mock_generate_gemini_response, mock_api_get_all_md_files

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
async def test_ingestion_flow(mock_integration_dependencies, mock_open):
    mock_ingestion_qdrant_client_instance, _, \
    mock_ingestion_embed_text, _, \
    _, mock_api_get_all_md_files = mock_integration_dependencies

    # Configure mock_api_get_all_md_files to return some files
    mock_api_get_all_md_files.return_value = [
        'C:/Users/user/Desktop/noor/frontend/docs/chapter1.md',
        'C:/Users/user/Desktop/noor/frontend/docs/chapter2.mdx',
    ]

    # Setup mock open to return content for each file
    mock_open.side_effect = [
        create_mock_md_file('chapter1.md', '# Chapter 1 Title\nContent of chapter 1.'),
        create_mock_md_file('chapter2.mdx', '## Chapter 2 Title\nContent of chapter 2.'),
    ]

    response = client.post("/ingestion/ingest")

    assert response.status_code == 201
    assert response.json() == {"status": "success", "message": "Ingested 2 documents."}

    mock_ingestion_qdrant_client_instance.recreate_collection.assert_called_once()
    assert mock_ingestion_embed_text.call_count == 2
    assert mock_ingestion_qdrant_client_instance.upsert.call_count == 2

    # Verify upsert calls
    call_args_1 = mock_ingestion_qdrant_client_instance.upsert.call_args_list[0].kwargs
    assert call_args_1['points'][0].payload['title'] == 'Chapter 1 Title'
    
    call_args_2 = mock_ingestion_qdrant_client_instance.upsert.call_args_list[1].kwargs
    assert call_args_2['points'][0].payload['title'] == 'Chapter 2 Title'


@pytest.mark.asyncio
async def test_chat_flow(mock_integration_dependencies):
    _, mock_chatbot_qdrant_client_instance, \
    _, mock_chatbot_embed_text, \
    mock_generate_gemini_response, _ = mock_integration_dependencies

    query = "What is Chapter 1 about?"
    session_id = "test-session-123"

    response = client.post("/chat", json={"query": query, "session_id": session_id})

    assert response.status_code == 200
    assert response.json() == {"response": "Mocked Gemini response."}

    mock_chatbot_embed_text.assert_called_once_with(query)
    mock_chatbot_qdrant_client_instance.search.assert_called_once()
    mock_generate_gemini_response.assert_called_once()
    
    # Verify the context passed to generate_gemini_response
    gen_response_args = mock_generate_gemini_response.call_args.args
    assert gen_response_args[0] == query # first arg is prompt
    assert gen_response_args[1][0]['title'] == "Context Title"
    assert gen_response_args[1][0]['content'] == "Context Content"

@pytest.mark.asyncio
async def test_selected_chat_flow(mock_integration_dependencies):
    _, _, \
    _, _, \
    mock_generate_gemini_response, _ = mock_integration_dependencies

    selected_text = "This is some selected text."
    query = "Explain this part."
    session_id = "test-session-456"

    response = client.post("/selected-chat", json={"text": selected_text, "query": query, "session_id": session_id})

    assert response.status_code == 200
    assert response.json() == {"response": "Mocked Gemini response."}

    mock_generate_gemini_response.assert_called_once()
    
    # Verify the context passed to generate_gemini_response
    gen_response_args = mock_generate_gemini_response.call_args.args
    assert gen_response_args[0] == query # first arg is prompt
    assert gen_response_args[1][0]['content'] == selected_text
