from pydantic import BaseModel, Field
from typing import Literal
import uuid
from datetime import datetime

class User(BaseModel):
    user_id: uuid.UUID = Field(default_factory=uuid.uuid4)
    email: str
    password_hash: str
    software_background: Literal["Beginner", "Intermediate", "Advanced"]
    hardware_experience: Literal["None", "Arduino/RPi", "ROS", "Robotics Kit"]
    preferred_learning: Literal["Visual", "Code-heavy", "Theory", "Hands-on"]
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
