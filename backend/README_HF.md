# Physical AI Backend

Backend API for Physical AI & Humanoid Robotics educational platform.

## Features

- **RAG Chatbot**: Question answering using Qdrant vector database and Groq LLM
- **Content Personalization**: Difficulty-based content filtering
- **Progress Tracking**: Track learning progress
- **Bookmarks**: Save favorite sections

## Deployment to Hugging Face Spaces

### Option 1: Using HF CLI

```bash
# Login to Hugging Face
huggingface-cli login

# Create a new Space
huggingface-cli repo create physical-ai-backend --type space --hardware cpu

# Clone the space
git clone https://huggingface.co/spaces/your-username/physical-ai-backend

# Copy backend files
cp -r backend/* physical-ai-backend/
cp backend/requirements.txt physical-ai-backend/
cp backend/hf_spaces.py physical-ai-backend/app.py

# Push to HF
cd physical-ai-backend
git add .
git commit -m "Deploy Physical AI backend"
git push
```

### Option 2: Using Python

```python
from huggingface_hub import HfApi, create_repo

api = HfApi(token="your_token_here")

# Create space if not exists
create_repo("your-username/physical-ai-backend", repo_type="space", space_sdk="docker")

# Upload files
api.upload_file(
    path_or_fileobj="backend/requirements.txt",
    path_in_repo="requirements.txt",
    repo_id="your-username/physical-ai-backend",
    repo_type="space"
)

api.upload_file(
    path_or_fileobj="backend/hf_spaces.py",
    path_in_repo="app.py",
    repo_id="your-username/physical-ai-backend",
    repo_type="space"
)
```

## Environment Variables

Set these in Space settings:

| Variable | Description |
|----------|-------------|
| `QDRANT_URL` | Qdrant cloud URL |
| `QDRANT_API_KEY` | Qdrant API key |
| `GROQ_API_KEY` | Groq API key |
| `GEMINI_API_KEY` | Google Gemini API key |
| `REDIS_URL` | Redis connection URL |

## API Endpoints

- `POST /api/chat` - Chat with curriculum
- `GET /health` - Health check
- `GET /` - API info

## Local Development

```bash
cd backend
pip install -r requirements.txt
python hf_spaces.py
```

## Connect Frontend

Update frontend `.env`:
```
API_BASE_URL=https://your-space.hf.space
```
