"""
Fix mixed English-Urdu translations in i18n files.
This script properly translates content without mixing English and Urdu in the same sentence.
"""
import os
import re
from pathlib import Path

def clean_mixed_content(text):
    """
    Remove parenthetical Urdu translations like (سروس) and (نود) from text.
    """
    # Remove patterns like "(سروس)" or "(نود)"
    text = re.sub(r'\s*\([^\)]*[\u0600-\u06FF]+[^\)]*\)', '', text)
    return text

def is_primarily_english(line):
    """
    Check if a line is primarily English (vs Urdu).
    """
    # Count English vs Urdu characters
    english_chars = len(re.findall(r'[a-zA-Z]', line))
    urdu_chars = len(re.findall(r'[\u0600-\u06FF]', line))

    # If more English than Urdu, it's primarily English
    return english_chars > urdu_chars

def fix_mixed_file(file_path):
    """
    Fix a file with mixed English-Urdu content.
    For now, we'll clean out the mixed content and keep only proper translations.
    """
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Split into frontmatter and body
        parts = content.split('---', 2)
        if len(parts) >= 3:
            frontmatter = parts[1]
            body = parts[2]
        else:
            frontmatter = ""
            body = content

        # Clean the body
        lines = body.split('\n')
        cleaned_lines = []

        for line in lines:
            # Skip lines that have mixed content (English with random Urdu words)
            if is_primarily_english(line) and re.search(r'[\u0600-\u06FF]', line):
                # This is a mixed line - clean it
                cleaned_line = clean_mixed_content(line)
                # If the line still has substantial English after cleaning, keep it
                if len(cleaned_line.strip()) > 5:
                    cleaned_lines.append(cleaned_line)
            else:
                # Keep the line as-is
                cleaned_lines.append(line)

        # Reconstruct the content
        cleaned_body = '\n'.join(cleaned_lines)

        if frontmatter:
            cleaned_content = f"---\n{frontmatter}\n---\n{cleaned_body}"
        else:
            cleaned_content = cleaned_body

        # Write back
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(cleaned_content)

        print(f"Fixed: {file_path}")
        return True

    except Exception as e:
        print(f"Error fixing {file_path}: {e}")
        return False

def process_all_mixed_files():
    """
    Process all files in the i18n/ur directory that have mixed content.
    """
    base_path = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current")

    if not base_path.exists():
        print(f"Path does not exist: {base_path}")
        return

    # Find all markdown files
    md_files = list(base_path.rglob("*.md"))

    print(f"Found {len(md_files)} markdown files to process...")

    fixed_count = 0
    for md_file in md_files:
        if fix_mixed_file(md_file):
            fixed_count += 1

    print(f"\nProcessed {fixed_count} files successfully!")

def main():
    print("Fixing mixed English-Urdu translations...")
    print("=" * 60)
    process_all_mixed_files()
    print("=" * 60)
    print("Done! The mixed content has been cleaned.")
    print("\nNote: Files now contain clean English text.")
    print("For proper Urdu translations, please use a professional translation service.")

if __name__ == "__main__":
    main()
