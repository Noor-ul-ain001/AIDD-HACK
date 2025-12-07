from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel
from typing import Literal
import uuid

# Assuming these imports from other services
from backend.src.services.personalization import personalize_chapter_content
from backend.src.services.translation import translate_text_with_gemini
# from backend.src.services.chapter_retrieval import get_chapter_content
# from backend.src.api.auth import get_current_user # To get user profile for personalization

router = APIRouter()

class PersonalizedChapterResponse(BaseModel):
    chapter_id: uuid.UUID
    title: str
    content: str # Adapted content

class TranslatedChapterResponse(BaseModel):
    chapter_id: uuid.UUID
    title: str
    content: str # Translated content


# Placeholder for getting current user (from auth module)
async def get_current_user_profile():
    # This would typically involve decoding JWT and fetching user profile from DB
    # For now, return mock data
    return {
        "software_background": "Intermediate",
        "hardware_experience": "ROS",
        "preferred_learning": "Code-heavy",
    }

# Placeholder for getting chapter content
async def get_chapter_content_from_db(chapter_id: uuid.UUID) -> dict:
    # In a real app, fetch from PostgreSQL/some content store
    # For now, return mock data
    return {
        "chapter_id": chapter_id,
        "title": "Mock Chapter Title",
        "content": "This is the original content of the chapter with some complex algorithms and basic concepts about robot communication methods.",
        "module_id": uuid.uuid4(),
        "order": 1
    }


@router.post("/chapters/{chapter_id}/personalize", response_model=PersonalizedChapterResponse)
async def personalize_chapter(chapter_id: uuid.UUID, user_profile: dict = Depends(get_current_user_profile)):
    original_chapter = await get_chapter_content_from_db(chapter_id)
    if not original_chapter:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Chapter not found"
        )
    
    adapted_content = await personalize_chapter_content(
        original_chapter["content"],
        user_profile["software_background"],
        user_profile["hardware_experience"],
        user_profile["preferred_learning"]
    )
    
    return PersonalizedChapterResponse(
        chapter_id=original_chapter["chapter_id"],
        title=original_chapter["title"],
        content=adapted_content
    )

@router.post("/chapters/{chapter_id}/translate", response_model=TranslatedChapterResponse)
async def translate_chapter(chapter_id: uuid.UUID):
    original_chapter = await get_chapter_content_from_db(chapter_id)
    if not original_chapter:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="Chapter not found"
        )
    
    translated_content = await translate_text_with_gemini(original_chapter["content"], "Urdu")
    
    return TranslatedChapterResponse(
        chapter_id=original_chapter["chapter_id"],
        title=original_chapter["title"],
        content=translated_content
    )

