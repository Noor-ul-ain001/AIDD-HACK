from fastapi import APIRouter
from pydantic import BaseModel
from typing import Optional
from backend.src.services.chatbot import get_chatbot_response, get_selected_text_chatbot_response

router = APIRouter()

class ChatRequest(BaseModel):
    query: str
    session_id: str

class SelectedChatRequest(BaseModel):
    text: str
    query: str
    session_id: str

class ChatResponse(BaseModel):
    response: str

@router.post("/chat", response_model=ChatResponse)
async def chat(request: ChatRequest):
    response_message = await get_chatbot_response(request.query, request.session_id)
    return ChatResponse(response=response_message)

@router.post("/selected-chat", response_model=ChatResponse)
async def selected_chat(request: SelectedChatRequest):
    response_message = await get_selected_text_chatbot_response(request.text, request.query, request.session_id)
    return ChatResponse(response=response_message)
