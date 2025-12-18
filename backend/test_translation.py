#!/usr/bin/env python3
"""
Test script to verify the translation functionality
"""
import sys
import asyncio
from pathlib import Path

# Add the src directory to the path so we can import modules
project_root = Path(__file__).parent
src_path = project_root / "src"
sys.path.insert(0, str(src_path))

from services.translation import TranslationService
from services.content_service import get_content_service

async def test_translation():
    print("Testing translation service...")
    
    try:
        # Get content for translation
        content_service = get_content_service()
        content = content_service.get_chapter_content("module-1/week-1-introduction")
        
        if not content:
            print("[ERROR] Could not retrieve content for translation")
            return
            
        print(f"[OK] Retrieved content, length: {len(content)} characters")
        print(f"First 100 chars: {content[:100]}")
        
        # Initialize translation service
        translation_service = TranslationService()
        print(f"[OK] Translation service initialized successfully")
        
        # Perform translation
        print("\nStarting translation to Urdu...")
        result = await translation_service.translate_content(content, "ur")
        
        print(f"[OK] Translation completed")
        print(f"Translated content length: {len(result.translated_content)} characters")
        print(f"Number of preserved code blocks: {len(result.preserved_code_blocks)}")
        print(f"Number of preserved links: {len(result.preserved_links)}")
        
        print("\nFirst 100 chars of translated content:")
        print(result.translated_content[:100])
        
        if len(result.translated_content) > 0 and "[TRANSLATION" not in result.translated_content[:50]:
            print("\n[SUCCESS] Translation appears to have worked properly!")
        else:
            print("\n[WARNING] Translation may not have worked properly - check content")
        
    except Exception as e:
        print(f"[ERROR] Error testing translation service: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    asyncio.run(test_translation())