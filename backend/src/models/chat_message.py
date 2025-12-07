from pydantic import BaseModel, Field
import uuid
from datetime import datetime

class ChatMessage(BaseModel):
    message_id: uuid.UUID = Field(default_factory=uuid.uuid4)
    user_id: uuid.UUID
    session_id: uuid.UUID
    query: str
    response: str
    timestamp: datetime = Field(default_factory=datetime.utcnow)
