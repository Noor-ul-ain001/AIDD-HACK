from fastapi import APIRouter, Depends, HTTPException
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from ..core.redis_client import get_redis_client
from ..services.rag import RAGService
from ..services.personalization import PersonalizationService, UserProfile, HardwareLevel, ExperienceLevel
from ..api.auth_enhanced import get_current_user_id
from ..core.database import get_async_db
from ..db.models import User
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
import json

router = APIRouter()

# Initialize services
rag_service = RAGService()
personalization_service = PersonalizationService()

# Pydantic models for personalization
class HardwareProfile(BaseModel):
    gpu: Optional[str] = None
    robotics_kit: Optional[str] = None
    experience_level: Optional[str] = None

class PersonalizeRequest(BaseModel):
    chapter_id: str
    hardware_profile: Optional[HardwareProfile] = None
    learning_goals: Optional[List[str]] = None

class PersonalizationRulesResponse(BaseModel):
    show_sections: List[str]
    hide_sections: List[str]
    code_complexity: str
    hardware_instructions: str

# Pydantic model for translation
class TranslateRequest(BaseModel):
    chapter_id: str
    target_language: str = "ur"  # Default to Urdu

@router.post("/translate")
async def translate_endpoint(
    request: TranslateRequest,
    user_id: str = Depends(get_current_user_id)
):
    """
    Translate chapter content to the specified language (currently Urdu).
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    from ..services.translation import TranslationService

    try:
        # Initialize translation service
        # In a real implementation, API credentials would come from settings
        translation_service = TranslationService()

        # For now, we'll return a mock translation result
        # In a real implementation, this would translate actual content
        translated_content = f"[URDU MOCK TRANSLATION] Content for chapter: {request.chapter_id}"
        preserved_code_blocks = []

        return {
            "translated_content": translated_content,
            "preserved_code_blocks": preserved_code_blocks
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Translation error: {str(e)}")

@router.post("/content/personalize", response_model=PersonalizationRulesResponse)
async def personalize_content(
    request: PersonalizeRequest,
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get personalization rules for a specific chapter based on user profile.
    Protected endpoint - requires valid JWT token in Authorization header.
    """
    try:
        # Get user profile from database to merge with request data
        result = await db.execute(select(User).filter(User.id == user_id))
        user = result.scalar_one_or_none()

        if not user:
            raise HTTPException(status_code=404, detail="User not found")

        # Use hardware profile from request if provided, otherwise use user's stored profile
        hardware_profile = request.hardware_profile or (
            user.hardware_profile if user.hardware_profile else {}
        )

        # Use learning goals from request if provided, otherwise use user's stored goals
        learning_goals = request.learning_goals or (
            user.learning_goals if user.learning_goals else []
        )

        # Determine hardware access level from the merged profile
        robotics_kit = hardware_profile.get("robotics_kit", "none") if isinstance(hardware_profile, dict) else "none"
        experience_level_str = hardware_profile.get("experience_level", "beginner") if isinstance(hardware_profile, dict) else "beginner"

        # Map the string values to our enum values
        if robotics_kit == "none":
            hardware_access = HardwareLevel.NONE
        elif robotics_kit == "jetson":
            hardware_access = HardwareLevel.JETSON
        else:
            hardware_access = HardwareLevel.FULL_ROBOT  # default to full robot if not specified properly

        if experience_level_str == "beginner":
            experience_level = ExperienceLevel.BEGINNER
        elif experience_level_str == "intermediate":
            experience_level = ExperienceLevel.INTERMEDIATE
        elif experience_level_str == "advanced":
            experience_level = ExperienceLevel.ADVANCED
        else:
            experience_level = ExperienceLevel.BEGINNER  # default to beginner

        # Create user profile
        user_profile = UserProfile(
            hardware_access=hardware_access,
            experience_level=experience_level,
            learning_goals=learning_goals
        )

        # Get personalization rules
        rules = personalization_service.get_personalization_rules(user_profile)

        return PersonalizationRulesResponse(
            show_sections=rules.show_sections,
            hide_sections=rules.hide_sections,
            code_complexity=rules.code_complexity,
            hardware_instructions=rules.hardware_instructions
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error generating personalization rules: {str(e)}")

# Caching for RAG responses (related to Task T044)
async def cache_response(cache_key: str, response_data: dict, ttl: int = 3600):
    """
    Cache RAG responses in Redis
    """
    redis_client = await get_redis_client()
    await redis_client.setex(
        cache_key,
        ttl,
        json.dumps(response_data)
    )

async def get_cached_response(cache_key: str):
    """
    Retrieve cached RAG response from Redis
    """
    redis_client = await get_redis_client()
    cached_data = await redis_client.get(cache_key)

    if cached_data:
        return json.loads(cached_data)

    return None