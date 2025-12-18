"""
Complete Urdu Translation Script for i18n Documentation
Uses deep-translator (free, no API key required)
Translates all English content to pure Urdu while preserving code blocks
"""

import os
import re
from pathlib import Path
from deep_translator import GoogleTranslator
import time

class UrduTranslator:
    def __init__(self):
        self.translator = GoogleTranslator(source='en', target='ur')
        self.code_blocks = []
        self.inline_code = []

    def preserve_code_blocks(self, content):
        """Extract and preserve code blocks with placeholders"""
        self.code_blocks = []
        self.inline_code = []

        # Preserve fenced code blocks (```)
        def replace_code_block(match):
            self.code_blocks.append(match.group(0))
            return f"\n___CODE_BLOCK_{len(self.code_blocks)-1}___\n"

        content = re.sub(r'```[\s\S]*?```', replace_code_block, content)

        # Preserve inline code (`)
        def replace_inline_code(match):
            self.inline_code.append(match.group(0))
            return f"___INLINE_CODE_{len(self.inline_code)-1}___"

        content = re.sub(r'`[^`\n]+`', replace_inline_code, content)

        # Preserve URLs
        content = re.sub(r'https?://[^\s]+', lambda m: f'___URL_{m.group(0)}___', content)

        return content

    def restore_code_blocks(self, content):
        """Restore code blocks and inline code from placeholders"""
        # Restore code blocks
        for i, block in enumerate(self.code_blocks):
            content = content.replace(f"___CODE_BLOCK_{i}___", block)

        # Restore inline code
        for i, code in enumerate(self.inline_code):
            content = content.replace(f"___INLINE_CODE_{i}___", code)

        # Restore URLs
        content = re.sub(r'___URL_(https?://[^\s]+)___', r'\1', content)

        return content

    def translate_text(self, text, max_length=4500):
        """Translate text to Urdu in chunks if needed"""
        if not text or not text.strip():
            return text

        # If text is too long, split into chunks
        if len(text) > max_length:
            chunks = []
            current_chunk = ""

            for line in text.split('\n'):
                if len(current_chunk) + len(line) + 1 > max_length:
                    if current_chunk:
                        chunks.append(current_chunk)
                    current_chunk = line
                else:
                    current_chunk += '\n' + line if current_chunk else line

            if current_chunk:
                chunks.append(current_chunk)

            translated_chunks = []
            for chunk in chunks:
                try:
                    translated = self.translator.translate(chunk)
                    translated_chunks.append(translated)
                    time.sleep(0.5)  # Rate limiting
                except Exception as e:
                    print(f"  [WARNING] Translation error: {str(e)}")
                    translated_chunks.append(chunk)

            return '\n'.join(translated_chunks)
        else:
            try:
                return self.translator.translate(text)
            except Exception as e:
                print(f"  [WARNING] Translation error: {str(e)}")
                return text

    def translate_markdown_content(self, content):
        """Translate markdown content while preserving structure"""
        # Preserve frontmatter
        frontmatter_match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)
        frontmatter = ""
        if frontmatter_match:
            frontmatter = frontmatter_match.group(0)
            content = content[len(frontmatter):]

        # Preserve code blocks
        content = self.preserve_code_blocks(content)

        # Split into sections to translate
        sections = []
        current_section = ""

        for line in content.split('\n'):
            # Check if line is a placeholder
            if '___CODE_BLOCK_' in line or '___INLINE_CODE_' in line or '___URL_' in line:
                if current_section.strip():
                    sections.append(('translate', current_section))
                    current_section = ""
                sections.append(('preserve', line))
            else:
                current_section += line + '\n'

        if current_section.strip():
            sections.append(('translate', current_section))

        # Translate sections
        translated_content = ""
        for section_type, section_text in sections:
            if section_type == 'translate':
                translated = self.translate_text(section_text.strip())
                if translated:
                    translated_content += translated + '\n'
            else:
                translated_content += section_text + '\n'

        # Restore code blocks
        translated_content = self.restore_code_blocks(translated_content)

        # Add frontmatter back
        return frontmatter + translated_content

    def translate_file(self, file_path):
        """Translate a single markdown file"""
        try:
            print(f"\n[TRANSLATING] {file_path.name}")

            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()

            # Check if already significantly translated
            urdu_chars = len(re.findall(r'[\u0600-\u06FF]', content))
            total_chars = len(content)

            if total_chars > 0 and urdu_chars > total_chars * 0.4:
                print(f"  [SKIP] Already translated (>{(urdu_chars/total_chars*100):.0f}% Urdu)")
                return False

            # Translate
            print(f"  [PROCESSING] Translating content...")
            translated = self.translate_markdown_content(content)

            # Backup original
            backup_path = file_path.with_suffix('.md.backup')
            if not backup_path.exists():
                with open(backup_path, 'w', encoding='utf-8') as f:
                    f.write(content)

            # Write translated content
            with open(file_path, 'w', encoding='utf-8') as f:
                f.write(translated)

            print(f"  [SUCCESS] Translation completed")
            return True

        except Exception as e:
            print(f"  [ERROR] {str(e)}")
            return False

def main():
    """Main translation function"""
    print("=" * 70)
    print("URDU TRANSLATION TOOL - i18n Documentation")
    print("=" * 70)
    print("\nThis script will translate all English content to pure Urdu")
    print("Code blocks and technical terms will be preserved\n")

    # Check if deep-translator is installed
    try:
        from deep_translator import GoogleTranslator
    except ImportError:
        print("[ERROR] deep-translator library not found!")
        print("\nPlease install it first:")
        print("  pip install deep-translator")
        return

    # Path to i18n documentation
    i18n_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\i18n\ur\docusaurus-plugin-content-docs\current')

    if not i18n_path.exists():
        print(f"[ERROR] Path not found: {i18n_path}")
        return

    # Find all markdown files
    md_files = sorted(i18n_path.rglob('*.md'))

    print(f"Found {len(md_files)} markdown files\n")
    print("=" * 70)

    translator = UrduTranslator()

    translated_count = 0
    skipped_count = 0
    error_count = 0

    for i, md_file in enumerate(md_files, 1):
        print(f"\n[{i}/{len(md_files)}] Processing: {md_file.relative_to(i18n_path)}")

        result = translator.translate_file(md_file)

        if result is True:
            translated_count += 1
        elif result is False:
            skipped_count += 1
        else:
            error_count += 1

        # Rate limiting - pause between files
        if i < len(md_files):
            time.sleep(1)

    print("\n" + "=" * 70)
    print("TRANSLATION COMPLETE")
    print("=" * 70)
    print(f"\nResults:")
    print(f"  ✓ Translated: {translated_count} files")
    print(f"  ⊘ Skipped:    {skipped_count} files (already translated)")
    print(f"  ✗ Errors:     {error_count} files")
    print(f"\nBackup files created with .md.backup extension")
    print("=" * 70)

if __name__ == '__main__':
    main()
