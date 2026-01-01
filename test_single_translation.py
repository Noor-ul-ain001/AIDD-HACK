"""
Test translation on a single file to verify it works correctly.
"""
import asyncio
import sys
sys.path.append('.')

from translate_markdown_files import translate_markdown_file

async def test_translation():
    """Test translation on intro.md"""
    input_file = "frontend/docs/intro.md"
    output_file = "frontend/i18n/ur/docusaurus-plugin-content-docs/current/intro.md"

    print(f"Testing translation on: {input_file}")
    print("=" * 60)

    success = await translate_markdown_file(input_file, output_file)

    if success:
        print("=" * 60)
        print("[SUCCESS] Translation test completed!")
        print(f"Check the output file: {output_file}")

        # Read and display first few lines
        with open(output_file, 'r', encoding='utf-8') as f:
            content = f.read()
            lines = content.split('\n')[:20]
            print("\nFirst 20 lines of translated content:")
            print("-" * 60)
            for line in lines:
                print(line)
    else:
        print("[FAILED] Translation test failed!")

if __name__ == "__main__":
    asyncio.run(test_translation())
