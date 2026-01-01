"""
Content service for the Physical AI & Humanoid Robotics platform.
Handles retrieval of documentation content for translation and other operations.
"""

import os
from pathlib import Path
from typing import Optional
import re
import frontmatter


class ContentService:
    def __init__(self, content_dir: str = "../frontend/docs"):
        """
        Initialize the content service.
        :param content_dir: Path to the documentation directory
        """
        self.content_dir = Path(content_dir)
        if not self.content_dir.exists():
            # If relative path doesn't work, try absolute path from project root
            project_root = Path(__file__).parent.parent.parent.parent
            self.content_dir = project_root / "frontend" / "docs"
        
        if not self.content_dir.exists():
            raise FileNotFoundError(f"Content directory does not exist: {self.content_dir}")

    def get_chapter_content(self, chapter_id: str) -> Optional[str]:
        """
        Retrieve content for a specific chapter by ID.
        
        Chapter IDs follow the pattern 'module-X/week-X-topic' or 'module-X/week-X-topic-slug'
        
        :param chapter_id: Chapter identifier (e.g., "module-1/week-1-introduction")
        :return: Chapter content as string or None if not found
        """
        # Convert chapter_id to possible file paths
        # e.g., "module-1/week-1-introduction" could be:
        # - frontend/docs/module-1/week-1-introduction.md
        # - frontend/docs/module-1/week-1-introduction/index.md
        
        # First, try the direct path with .md extension
        direct_path = self.content_dir / f"{chapter_id}.md"
        if direct_path.exists():
            return self._read_content_file(direct_path)
        
        # Second, try as directory with index.md
        dir_path = self.content_dir / chapter_id
        index_path = dir_path / "index.md"
        if index_path.exists():
            return self._read_content_file(index_path)
        
        # Third, split the chapter_id and look in the appropriate module directory
        parts = chapter_id.split("/")
        if len(parts) >= 2:
            module_dir = self.content_dir / parts[0]  # e.g., module-1
            possible_files = [
                module_dir / f"{parts[1]}.md",  # week-1-introduction.md
                module_dir / f"{parts[1]}.mdx",  # if using .mdx
            ]
            
            for file_path in possible_files:
                if file_path.exists():
                    return self._read_content_file(file_path)
        
        # If no file found, return None
        return None

    def _read_content_file(self, file_path: Path) -> str:
        """
        Read content from a file, extracting just the content part (not frontmatter).
        """
        try:
            with open(file_path, 'r', encoding='utf-8') as file:
                post = frontmatter.load(file)
                return post.content
        except Exception as e:
            print(f"Error reading file {file_path}: {str(e)}")
            return None

    def get_all_chapters(self) -> list:
        """
        Get a list of all available chapter IDs.
        """
        chapters = []
        
        for module_dir in self.content_dir.iterdir():
            if module_dir.is_dir() and module_dir.name.startswith("module-"):
                for content_file in module_dir.glob("*.md"):
                    # Create chapter ID from relative path
                    relative_path = content_file.relative_to(self.content_dir)
                    chapter_id = str(relative_path.with_suffix(''))
                    chapters.append(chapter_id)
                    
                # Also check for subdirectories (like week directories)
                for sub_dir in module_dir.iterdir():
                    if sub_dir.is_dir():
                        for sub_content_file in sub_dir.glob("*.md"):
                            relative_path = sub_content_file.relative_to(self.content_dir)
                            chapter_id = str(relative_path.with_suffix(''))
                            chapters.append(chapter_id)
        
        return chapters


# Global instance of the content service
content_service = ContentService()


def get_content_service() -> ContentService:
    """
    Get the content service instance.
    """
    return content_service