"""
Fast translation for remaining 36 files
"""
import os
import re
import time
from pathlib import Path
from deep_translator import GoogleTranslator

class FastUrduTranslator:
    def __init__(self):
        self.translator = GoogleTranslator(source='en', target='ur')
        self.i18n_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\i18n\ur\docusaurus-plugin-content-docs\current')
        self.code_blocks = []
        self.inline_code = []

    def preserve_code(self, content):
        """Preserve code blocks and inline code"""
        self.code_blocks = []
        self.inline_code = []

        # Preserve fenced code blocks
        def replace_code_block(match):
            self.code_blocks.append(match.group(0))
            return f"\n___CB{len(self.code_blocks)-1}___\n"

        content = re.sub(r'```[\s\S]*?```', replace_code_block, content)

        # Preserve inline code
        def replace_inline(match):
            self.inline_code.append(match.group(0))
            return f"___IC{len(self.inline_code)-1}___"

        content = re.sub(r'`[^`\n]+`', replace_inline, content)

        # Preserve URLs
        content = re.sub(r'https?://[^\s]+', lambda m: f'___URL{m.group(0)}___', content)

        return content

    def restore_code(self, content):
        """Restore preserved code"""
        for i, block in enumerate(self.code_blocks):
            content = content.replace(f"___CB{i}___", block)

        for i, code in enumerate(self.inline_code):
            content = content.replace(f"___IC{i}___", code)

        content = re.sub(r'___URL(https?://[^\s]+)___', r'\1', content)

        return content

    def translate_text(self, text):
        """Translate text with error handling"""
        if not text or not text.strip():
            return text

        try:
            # Split into smaller chunks if too long
            if len(text) > 4500:
                chunks = []
                lines = text.split('\n')
                current_chunk = ""

                for line in lines:
                    if len(current_chunk) + len(line) + 1 > 4500:
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
                        time.sleep(0.3)
                    except:
                        translated_chunks.append(chunk)

                return '\n'.join(translated_chunks)
            else:
                return self.translator.translate(text)
        except Exception as e:
            print(f"  [WARN] Translation failed: {e}")
            return text

    def translate_markdown(self, content):
        """Translate markdown preserving structure"""
        # Preserve frontmatter
        frontmatter = ""
        if content.startswith('---\n'):
            end = content.find('\n---\n', 4)
            if end != -1:
                frontmatter = content[:end+5]
                content = content[end+5:]

        # Preserve code
        content = self.preserve_code(content)

        # Split by preserved elements
        sections = []
        current = ""

        for line in content.split('\n'):
            if '___CB' in line or '___IC' in line or '___URL' in line:
                if current.strip():
                    sections.append(('translate', current))
                    current = ""
                sections.append(('preserve', line))
            else:
                current += line + '\n'

        if current.strip():
            sections.append(('translate', current))

        # Translate
        result = ""
        for section_type, section_text in sections:
            if section_type == 'translate':
                translated = self.translate_text(section_text.strip())
                if translated:
                    result += translated + '\n'
            else:
                result += section_text + '\n'

        # Restore code
        result = self.restore_code(result)

        return frontmatter + result

    def translate_files(self, files_to_translate):
        """Translate list of files"""
        total = len(files_to_translate)
        translated = 0

        for i, rel_path in enumerate(files_to_translate, 1):
            file_path = self.i18n_path / rel_path

            print(f"\n[{i}/{total}] {rel_path}")

            try:
                # Read
                with open(file_path, 'r', encoding='utf-8') as f:
                    content = f.read()

                # Backup
                backup = file_path.with_suffix('.md.backup')
                if not backup.exists():
                    with open(backup, 'w', encoding='utf-8') as f:
                        f.write(content)

                # Translate
                print("  [TRANSLATING]...", end="", flush=True)
                translated_content = self.translate_markdown(content)

                # Save
                with open(file_path, 'w', encoding='utf-8') as f:
                    f.write(translated_content)

                print(" [DONE]")
                translated += 1
                time.sleep(0.5)

            except Exception as e:
                print(f"  [ERROR] {str(e)}")

        return translated

def main():
    print("="*70)
    print("FAST URDU TRANSLATION FOR REMAINING FILES")
    print("="*70)

    # Files that need translation
    files_to_translate = [
        r'module-1\week-1-introduction\1-3-installation-setup.md',
        r'module-1\week-1-introduction\1-4-practical-exercises.md',
        r'module-1\week-2-nodes-topics\2-1-understanding-nodes.md',
        r'module-1\week-2-nodes-topics\2-2-working-with-topics.md',
        r'module-1\week-2-nodes-topics\2-3-topic-design-patterns.md',
        r'module-1\week-2-nodes-topics\2-4-practical-topics-exercises.md',
        r'module-1\week-2-nodes-topics.md',
        r'module-1\week-3-services-actions\3-1-understanding-services.md',
        r'module-1\week-3-services-actions\3-2-understanding-actions.md',
        r'module-1\week-3-services-actions\3-3-comparison-topic-service-action.md',
        r'module-1\week-3-services-actions\3-4-communication-practical-exercises.md',
        r'module-1\week-3-services-actions.md',
        r'module-1\week-4-tf-urdf\4-1-understanding-tf2.md',
        r'module-1\week-4-tf-urdf\4-2-understanding-urdf.md',
        r'module-1\week-4-tf-urdf\4-3-integration-tf2-urdf.md',
        r'module-1\week-4-tf-urdf\4-4-tf2-urdf-practical-exercises.md',
        r'module-1\week-4-tf-urdf.md',
        r'module-2\week-1-introduction\2-2-gazebo-installation-setup.md',
        r'module-2\week-1-introduction\2-3-basic-simulation-concepts.md',
        r'module-2\week-1-introduction\2-4-simulation-practical-exercises.md',
        r'module-2\week-2-gazebo-basics\3-1-simulation-integration-techniques.md',
        r'module-2\week-2-gazebo-basics\4-1-advanced-simulation-techniques.md',
        r'module-2\week-3-simulation-integration\3-1-simulation-integration-techniques.md',
        r'module-2\week-4-advanced-simulation\4-1-advanced-simulation-techniques.md',
        r'module-3\week-1-introduction\3-2-isaac-sim-installation-setup.md',
        r'module-3\week-1-introduction\3-3-isaac-sim-basics-robotics.md',
        r'module-3\week-1-introduction\3-4-isaac-sim-practical-exercises.md',
        r'module-3\week-2-isaac-ros-basics\2-1-introduction-to-isaac-ros.md',
        r'module-3\week-3-advanced-isaac-sim\3-1-advanced-isaac-sim-techniques.md',
        r'module-3\week-4-isaac-sim-applications\4-1-isaac-sim-applications.md',
        r'module-4\week-1-introduction\4-2-vla-architecture-deep-learning.md',
        r'module-4\week-1-introduction\4-3-vla-training-data-collection.md',
        r'module-4\week-1-introduction\4-4-vla-practical-implementation.md',
        r'module-4\week-2-vla-fundamentals\2-1-introduction-to-vla-models.md',
        r'module-4\week-3-vla-integration\3-1-vla-integration-with-robotics.md',
        r'module-4\week-4-advanced-vla-applications\4-1-advanced-vla-applications.md'
    ]

    translator = FastUrduTranslator()
    translated = translator.translate_files(files_to_translate)

    print("\n" + "="*70)
    print("TRANSLATION COMPLETE")
    print("="*70)
    print(f"\nTranslated: {translated}/{len(files_to_translate)} files")
    print("="*70)

if __name__ == '__main__':
    main()
