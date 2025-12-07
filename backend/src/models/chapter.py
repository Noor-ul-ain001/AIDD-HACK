from pydantic import BaseModel, Field
import uuid

class Chapter(BaseModel):
    chapter_id: uuid.UUID = Field(default_factory=uuid.uuid4)
    module_id: uuid.UUID
    title: str
    content: str
    order: int
