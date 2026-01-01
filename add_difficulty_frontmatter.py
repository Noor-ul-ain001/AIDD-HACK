"""
Script to add difficulty frontmatter to all markdown files that don't have it.
Module 1: beginner
Module 2: intermediate
Module 3: intermediate
Module 4: advanced
"""

import os
import re
from pathlib import Path

# Define difficulty levels for each module
DIFFICULTY_MAP = {
    'module-1': 'beginner',
    'module-2': 'intermediate',
    'module-3': 'intermediate',
    'module-4': 'advanced',
}

def get_module_from_path(file_path):
    """Extract module number from file path"""
    for module in DIFFICULTY_MAP.keys():
        if module in file_path:
            return module
    return None

def has_difficulty_frontmatter(content):
    """Check if file already has difficulty in frontmatter"""
    return 'difficulty:' in content

def add_difficulty_to_frontmatter(content, difficulty):
    """Add difficulty to existing frontmatter or create new frontmatter"""
    lines = content.split('\n')

    # Check if file has frontmatter
    if lines[0].strip() == '---':
        # Find end of frontmatter
        end_index = -1
        for i in range(1, len(lines)):
            if lines[i].strip() == '---':
                end_index = i
                break

        if end_index > 0:
            # Insert difficulty before closing ---
            lines.insert(end_index, f'difficulty: {difficulty}')
            return '\n'.join(lines)

    # No frontmatter, create one
    new_frontmatter = f"""---
sidebar_position: 1
difficulty: {difficulty}
---

"""
    return new_frontmatter + content

def process_markdown_files(docs_dir):
    """Process all markdown files in the docs directory"""
    docs_path = Path(docs_dir)
    processed_count = 0
    skipped_count = 0

    for md_file in docs_path.rglob('*.md'):
        # Skip intro.md as it's the main introduction page
        if md_file.name == 'intro.md':
            continue

        file_path_str = str(md_file)
        module = get_module_from_path(file_path_str)

        if not module:
            print(f"Skipping {md_file.name} - no module found in path")
            skipped_count += 1
            continue

        difficulty = DIFFICULTY_MAP[module]

        # Read file content
        with open(md_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Check if already has difficulty
        if has_difficulty_frontmatter(content):
            print(f"[OK] {md_file.name} already has difficulty set")
            skipped_count += 1
            continue

        # Add difficulty to frontmatter
        new_content = add_difficulty_to_frontmatter(content, difficulty)

        # Write back to file
        with open(md_file, 'w', encoding='utf-8') as f:
            f.write(new_content)

        print(f"[ADDED] difficulty: {difficulty} to {md_file.name}")
        processed_count += 1

    print(f"\n[DONE] Processed {processed_count} files")
    print(f"[SKIP] Skipped {skipped_count} files (already had difficulty or no module)")

if __name__ == '__main__':
    docs_directory = r'C:\Users\user\Desktop\noor-ana - Copy\frontend\docs'
    print(f"Processing markdown files in: {docs_directory}\n")
    process_markdown_files(docs_directory)
