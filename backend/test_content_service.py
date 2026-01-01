#!/usr/bin/env python3
"""
Test script to verify the content service functionality
"""
import sys
from pathlib import Path

# Add the src directory to the path so we can import modules
project_root = Path(__file__).parent
src_path = project_root / "src"
sys.path.insert(0, str(src_path))

from services.content_service import get_content_service

def test_content_service():
    print("Testing content service...")
    
    try:
        content_service = get_content_service()
        print(f"[OK] Content service initialized successfully")
        print(f"Content directory: {content_service.content_dir}")
        print(f"Content directory exists: {content_service.content_dir.exists()}")

        all_chapters = content_service.get_all_chapters()
        print(f"[OK] Found {len(all_chapters)} chapters:")
        for chapter in all_chapters[:10]:  # Show first 10
            print(f"  - {chapter}")

        # Test getting a specific chapter
        test_chapter = "module-1/week-1-introduction"
        print(f"\nTrying to get content for: {test_chapter}")
        content = content_service.get_chapter_content(test_chapter)

        if content:
            print(f"[OK] Successfully retrieved content for {test_chapter}")
            print(f"Content length: {len(content)} characters")
            print(f"First 100 chars: {content[:100]}")
        else:
            print(f"[ERROR] Content not found for {test_chapter}")

        # Test a few more possible chapter IDs from the actual files
        possible_ids = [
            "module-1/week-1-introduction",  # week-1-introduction.md
            "module-1/week-2-nodes-topics",  # week-2-nodes-topics.md
            "module-1/week-3-services-actions",  # week-3-services-actions.md
            "module-1/week-4-tf-urdf",  # week-4-tf-urdf.md
        ]

        print("\nTesting possible chapter IDs:")
        for chapter_id in possible_ids:
            content = content_service.get_chapter_content(chapter_id)
            if content:
                print(f"[OK] {chapter_id}: {len(content)} chars")
            else:
                print(f"[ERROR] {chapter_id}: Not found")

        # Test subdirectory content (if any)
        subdirs = ["week-1-introduction", "week-2-nodes-topics"]
        for subdir in subdirs:
            chapter_id = f"module-1/{subdir}"
            content = content_service.get_chapter_content(chapter_id)
            if content:
                print(f"[OK] {chapter_id}: {len(content)} chars")

    except Exception as e:
        print(f"[ERROR] Error testing content service: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    test_content_service()