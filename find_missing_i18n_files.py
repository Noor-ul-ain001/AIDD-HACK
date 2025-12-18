"""
Find missing files in i18n/ur folder compared to docs folder
"""

from pathlib import Path

# Paths
docs_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\docs')
i18n_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\i18n\ur\docusaurus-plugin-content-docs\current')

# Get all markdown files from docs
docs_files = set()
for md_file in docs_path.rglob('*.md'):
    rel_path = md_file.relative_to(docs_path)
    docs_files.add(str(rel_path).replace('\\', '/'))

# Get all markdown files from i18n
i18n_files = set()
for md_file in i18n_path.rglob('*.md'):
    rel_path = md_file.relative_to(i18n_path)
    i18n_files.add(str(rel_path).replace('\\', '/'))

# Find missing files
missing_files = docs_files - i18n_files

print(f"Total files in docs: {len(docs_files)}")
print(f"Total files in i18n/ur: {len(i18n_files)}")
print(f"Missing files: {len(missing_files)}\n")

if missing_files:
    print("Missing files that need to be translated:")
    for file in sorted(missing_files):
        print(f"  - {file}")
else:
    print("All files are present in i18n folder!")
