from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, Field
from typing import Literal, Optional
from datetime import datetime, timedelta
import uuid

# Placeholder for Better Auth library imports
# from better_auth import create_user, authenticate_user, generate_jwt_token
# from better_auth.schemas import UserCreate, UserLogin, Token

# Assuming these are available from better_auth or custom implementation
class UserCreate(BaseModel):
    email: str
    password: str
    software_background: Literal["Beginner", "Intermediate", "Advanced"]
    hardware_experience: Literal["None", "Arduino/RPi", "ROS", "Robotics Kit"]
    preferred_learning: Literal["Visual", "Code-heavy", "Theory", "Hands-on"]

class UserLogin(BaseModel):
    email: str
    password: str

class Token(BaseModel):
    access_token: str
    token_type: str

router = APIRouter()

# Placeholder for user database/storage
fake_users_db = {}

def get_password_hash(password: str) -> str:
    # In a real application, use a proper hashing library like Passlib
    return f"hashed_{password}"

def verify_password(plain_password: str, hashed_password: str) -> bool:
    # In a real application, use a proper hashing library like Passlib
    return f"hashed_{plain_password}" == hashed_password

def create_access_token(data: dict, expires_delta: Optional[timedelta] = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15) # Default expiration
    to_encode.update({"exp": expire})
    # This is a placeholder for JWT encoding
    return f"mock_jwt_token_for_{to_encode['sub']}"

@router.post("/signup", response_model=Token)
async def signup(user: UserCreate):
    if user.email in fake_users_db:
        raise HTTPException(
            status_code=status.HTTP_400_BAD_REQUEST,
            detail="Email already registered"
        )
    
    hashed_password = get_password_hash(user.password)
    new_user_id = str(uuid.uuid4())
    fake_users_db[user.email] = {
        "user_id": new_user_id,
        "email": user.email,
        "hashed_password": hashed_password,
        "software_background": user.software_background,
        "hardware_experience": user.hardware_experience,
        "preferred_learning": user.preferred_learning,
        "created_at": datetime.utcnow(),
        "updated_at": datetime.utcnow()
    }

    access_token_expires = timedelta(minutes=30)
    access_token = create_access_token(
        data={"sub": user.email, "user_id": new_user_id}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}

@router.post("/login", response_model=Token)
async def login(user_credentials: UserLogin):
    user_data = fake_users_db.get(user_credentials.email)
    if not user_data or not verify_password(user_credentials.password, user_data["hashed_password"]):
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Incorrect username or password",
            headers={"WWW-Authenticate": "Bearer"},
        )
    
    access_token_expires = timedelta(minutes=30)
    access_token = create_access_token(
        data={"sub": user_credentials.email, "user_id": user_data["user_id"]}, expires_delta=access_token_expires
    )
    return {"access_token": access_token, "token_type": "bearer"}
