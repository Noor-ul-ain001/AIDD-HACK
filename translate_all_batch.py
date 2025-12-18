"""
Batch translate all markdown files with progress tracking.
"""
import asyncio
import sys
from pathlib import Path
from translate_markdown_files import translate_markdown_file

async def translate_all_files():
    """Translate all markdown files in batches."""

    source_base = Path("frontend/docs")
    target_base = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current")

    # Get all markdown files
    md_files = list(source_base.rglob("*.md"))

    print(f"Found {len(md_files)} files to translate")
    print("=" * 70)

    success_count = 0
    failed_count = 0

    for i, md_file in enumerate(md_files, 1):
        rel_path = md_file.relative_to(source_base)
        output_file = target_base / rel_path

        print(f"[{i}/{len(md_files)}] Translating: {rel_path}")

        try:
            if await translate_markdown_file(str(md_file), str(output_file)):
                success_count += 1
            else:
                failed_count += 1
        except Exception as e:
            print(f"   [ERROR] {e}")
            failed_count += 1

        # Small delay to avoid rate limits
        await asyncio.sleep(2)

        # Progress update every 5 files
        if i % 5 == 0:
            print(f"   Progress: {success_count} successful, {failed_count} failed")
            print("-" * 70)

    print("=" * 70)
    print(f"COMPLETE!")
    print(f"Success: {success_count}/{len(md_files)}")
    print(f"Failed: {failed_count}/{len(md_files)}")

if __name__ == "__main__":
    print("Starting batch translation...")
    print("This will take approximately 5-10 minutes for all files.\n")
    asyncio.run(translate_all_files())
