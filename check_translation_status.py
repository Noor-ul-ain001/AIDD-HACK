"""
Check which files need translation
"""
import re
from pathlib import Path

i18n_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\i18n\ur\docusaurus-plugin-content-docs\current')

needs_translation = []
already_translated = []

for md_file in sorted(i18n_path.rglob('*.md')):
    try:
        content = md_file.read_text(encoding='utf-8')
        urdu_chars = len(re.findall(r'[\u0600-\u06FF]', content))
        total_chars = len(content)

        if total_chars > 0:
            urdu_percent = (urdu_chars / total_chars) * 100

            rel_path = md_file.relative_to(i18n_path)

            if urdu_percent < 40:
                needs_translation.append(str(rel_path))
            else:
                already_translated.append(str(rel_path))
    except Exception as e:
        print(f"Error reading {md_file}: {e}")

print(f"Summary:")
print(f"  Already translated: {len(already_translated)} files")
print(f"  Need translation: {len(needs_translation)} files")
print(f"  Total: {len(already_translated) + len(needs_translation)} files")

if needs_translation:
    print(f"\nFiles needing translation ({len(needs_translation)}):")
    for file in needs_translation:
        print(f"  - {file}")
else:
    print("\nAll files are translated!")
