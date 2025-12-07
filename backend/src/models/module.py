from pydantic import BaseModel, Field
import uuid

class Module(BaseModel):
    module_id: uuid.UUID = Field(default_factory=uuid.uuid4)
    title: str
    description: str
    order: int
