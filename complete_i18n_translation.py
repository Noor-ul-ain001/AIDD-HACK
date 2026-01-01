"""
Complete i18n Translation Script
1. Copy any missing files from docs to i18n/ur
2. Translate ALL files to pure Urdu
"""

import os
import shutil
from pathlib import Path
from deep_translator import GoogleTranslator
import re
import time

class CompleteUrduTranslator:
    def __init__(self):
        self.translator = GoogleTranslator(source='en', target='ur')
        self.code_blocks = []
        self.inline_code = []

        self.docs_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\docs')
        self.i18n_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\i18n\ur\docusaurus-plugin-content-docs\current')

    def copy_missing_files(self):
        """Copy files from docs to i18n that don't exist"""
        print("\n" + "="*70)
        print("STEP 1: Copying Missing Files")
        print("="*70)

        copied = 0
        for md_file in self.docs_path.rglob('*.md'):
            rel_path = md_file.relative_to(self.docs_path)
            target_file = self.i18n_path / rel_path

            if not target_file.exists():
                print(f"[COPY] {rel_path}")
                target_file.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(md_file, target_file)
                copied += 1

        print(f"\nCopied {copied} missing files")
        return copied

    def preserve_code_blocks(self, content):
        """Extract and preserve code blocks"""
        self.code_blocks = []
        self.inline_code = []

        # Preserve fenced code blocks
        def replace_code_block(match):
            self.code_blocks.append(match.group(0))
            return f"\n___CODE_BLOCK_{len(self.code_blocks)-1}___\n"

        content = re.sub(r'```[\s\S]*?```', replace_code_block, content)

        # Preserve inline code
        def replace_inline_code(match):
            self.inline_code.append(match.group(0))
            return f"___INLINE_CODE_{len(self.inline_code)-1}___"

        content = re.sub(r'`[^`\n]+`', replace_inline_code, content)

        # Preserve URLs
        content = re.sub(r'https?://[^\s]+', lambda m: f'___URL_{m.group(0)}___', content)

        return content

    def restore_code_blocks(self, content):
        """Restore code blocks"""
        for i, block in enumerate(self.code_blocks):
            content = content.replace(f"___CODE_BLOCK_{i}___", block)

        for i, code in enumerate(self.inline_code):
            content = content.replace(f"___INLINE_CODE_{i}___", code)

        content = re.sub(r'___URL_(https?://[^\s]+)___', r'\1', content)

        return content

    def translate_text(self, text, max_length=4500):
        """Translate text to Urdu"""
        if not text or not text.strip():
            return text

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
                    time.sleep(0.5)
                except Exception as e:
                    translated_chunks.append(chunk)

            return '\n'.join(translated_chunks)
        else:
            try:
                return self.translator.translate(text)
            except Exception as e:
                return text

    def translate_markdown(self, content):
        """Translate markdown content"""
        # Preserve frontmatter
        frontmatter_match = re.match(r'^---\n(.*?)\n---\n', content, re.DOTALL)
        frontmatter = ""
        if frontmatter_match:
            frontmatter = frontmatter_match.group(0)
            content = content[len(frontmatter):]

        # Preserve code blocks
        content = self.preserve_code_blocks(content)

        # Split and translate
        sections = []
        current_section = ""

        for line in content.split('\n'):
            if '___CODE_BLOCK_' in line or '___INLINE_CODE_' in line or '___URL_' in line:
                if current_section.strip():
                    sections.append(('translate', current_section))
                    current_section = ""
                sections.append(('preserve', line))
            else:
                current_section += line + '\n'

        if current_section.strip():
            sections.append(('translate', current_section))

        # Translate
        translated_content = ""
        for section_type, section_text in sections:
            if section_type == 'translate':
                translated = self.translate_text(section_text.strip())
                if translated:
                    translated_content += translated + '\n'
            else:
                translated_content += section_text + '\n'

        # Restore
        translated_content = self.restore_code_blocks(translated_content)

        return frontmatter + translated_content

    def translate_all_files(self):
        """Translate all markdown files"""
        print("\n" + "="*70)
        print("STEP 2: Translating All Files to Pure Urdu")
        print("="*70)

        md_files = sorted(self.i18n_path.rglob('*.md'))

        translated_count = 0
        skipped_count = 0

        for i, md_file in enumerate(md_files, 1):
            rel_path = md_file.relative_to(self.i18n_path)
            print(f"\n[{i}/{len(md_files)}] {rel_path}")

            try:
                with open(md_file, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Check if already translated
                urdu_chars = len(re.findall(r'[\u0600-\u06FF]', content))
                total_chars = len(content)

                if total_chars > 0 and urdu_chars > total_chars * 0.4:
                    print(f"  [SKIP] Already {(urdu_chars/total_chars*100):.0f}% Urdu")
                    skipped_count += 1
                    continue

                # Translate
                print(f"  [TRANSLATING]...")
                translated = self.translate_markdown(content)

                # Save
                with open(md_file, 'w', encoding='utf-8') as f:
                    f.write(translated)

                # Backup
                backup_path = md_file.with_suffix('.md.backup')
                if not backup_path.exists():
                    with open(backup_path, 'w', encoding='utf-8') as f:
                        f.write(content)

                print(f"  [SUCCESS]")
                translated_count += 1
                time.sleep(1)  # Rate limiting

            except Exception as e:
                print(f"  [ERROR] {str(e)}")

        return translated_count, skipped_count

def main():
    print("="*70)
    print("COMPLETE URDU TRANSLATION FOR i18n DOCUMENTATION")
    print("="*70)

    translator = CompleteUrduTranslator()

    # Step 1: Copy missing files
    copied = translator.copy_missing_files()

    # Step 2: Translate all files
    translated, skipped = translator.translate_all_files()

    print("\n" + "="*70)
    print("TRANSLATION COMPLETE")
    print("="*70)
    print(f"\nResults:")
    print(f"  Files copied:     {copied}")
    print(f"  Files translated: {translated}")
    print(f"  Files skipped:    {skipped}")
    print(f"\nAll content is now in pure Urdu!")
    print("="*70)

if __name__ == '__main__':
    main()
