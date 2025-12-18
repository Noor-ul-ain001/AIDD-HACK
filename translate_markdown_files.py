"""
Properly translate markdown files from English to Urdu using Groq API.
This uses the same translation service as the backend.
"""
import os
import re
import asyncio
from pathlib import Path
from groq import Groq

# Initialize Groq client
GROQ_API_KEY = os.getenv("GROQ_API_KEY")
if not GROQ_API_KEY:
    raise ValueError("GROQ_API_KEY environment variable not set")
client = Groq(api_key=GROQ_API_KEY)

def extract_code_blocks(content):
    """Extract code blocks to preserve them during translation."""
    code_blocks = []
    pattern = r'```[\s\S]*?```'

    def replacer(match):
        code_blocks.append(match.group(0))
        return f"__CODE_BLOCK_{len(code_blocks)-1}__"

    content_with_placeholders = re.sub(pattern, replacer, content)
    return code_blocks, content_with_placeholders

def restore_code_blocks(content, code_blocks):
    """Restore code blocks after translation."""
    for i, block in enumerate(code_blocks):
        content = content.replace(f"__CODE_BLOCK_{i}__", block)
    return content

async def translate_with_groq(content, target_language="ur"):
    """Translate content using Groq API."""

    # Create a detailed prompt
    prompt = f"""Translate the following educational content about robotics and AI to Urdu ({target_language}).

IMPORTANT RULES:
1. Translate ALL English text to proper, natural Urdu
2. Keep code blocks (marked as __CODE_BLOCK_N__) unchanged
3. Keep markdown formatting (like ##, ###, **, etc.)
4. For technical terms, use appropriate Urdu transliteration:
   - ROS 2 → ROS 2 (keep as is)
   - Node → نوڈ
   - Topic → ٹاپک
   - Service → سروس
   - Action → ایکشن
   - Publisher → پبلشر
   - Subscriber → سبسکرائیبر
   - Package → پیکیج
   - Gazebo → گیزبو
   - Isaac Sim → آئزک سیم
5. Translate sentences completely - do NOT mix English and Urdu in the same sentence
6. Use natural Urdu sentence structure
7. Keep URLs and links in their original form

Content to translate:
{content}

Provide ONLY the translated text, with no explanations or additional commentary."""

    try:
        # Call Groq API
        chat_completion = client.chat.completions.create(
            messages=[
                {
                    "role": "system",
                    "content": "You are an expert translator specializing in technical content translation from English to Urdu. You provide accurate, natural translations while preserving technical terminology and formatting."
                },
                {
                    "role": "user",
                    "content": prompt
                }
            ],
            model="llama-3.3-70b-versatile",
            temperature=0.3,
            max_tokens=8000
        )

        translated_text = chat_completion.choices[0].message.content
        return translated_text.strip()

    except Exception as e:
        print(f"Error during translation: {e}")
        return content  # Return original if translation fails

async def translate_markdown_file(input_file, output_file):
    """Translate a markdown file from English to Urdu."""

    try:
        # Read the file
        with open(input_file, 'r', encoding='utf-8') as f:
            content = f.read()

        # Split frontmatter and body
        parts = content.split('---', 2)
        if len(parts) >= 3:
            frontmatter = parts[1]
            body = parts[2]
        else:
            frontmatter = ""
            body = content

        # Extract code blocks
        code_blocks, body_with_placeholders = extract_code_blocks(body)

        # Translate the body
        print(f"Translating: {input_file}")
        translated_body = await translate_with_groq(body_with_placeholders, "ur")

        # Restore code blocks
        final_body = restore_code_blocks(translated_body, code_blocks)

        # Reconstruct the file
        if frontmatter:
            final_content = f"---{frontmatter}---\n{final_body}"
        else:
            final_content = final_body

        # Write output
        os.makedirs(os.path.dirname(output_file), exist_ok=True)
        with open(output_file, 'w', encoding='utf-8') as f:
            f.write(final_content)

        print(f"[OK] Translated: {output_file}")
        return True

    except Exception as e:
        print(f"[ERROR] Error translating {input_file}: {e}")
        return False

async def translate_all_files():
    """Translate all markdown files in the i18n directory."""

    source_base = Path("frontend/docs")
    target_base = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current")

    # Get all markdown files from source
    md_files = list(source_base.rglob("*.md"))

    print(f"Found {len(md_files)} markdown files to translate")
    print("=" * 60)

    success_count = 0
    for md_file in md_files:
        # Calculate relative path
        rel_path = md_file.relative_to(source_base)
        output_file = target_base / rel_path

        # Translate the file
        if await translate_markdown_file(str(md_file), str(output_file)):
            success_count += 1

        # Add a small delay to avoid rate limits
        await asyncio.sleep(1)

    print("=" * 60)
    print(f"Successfully translated {success_count}/{len(md_files)} files")

async def main():
    """Main function."""
    print("Starting translation of markdown files to Urdu...")
    print("This may take a while as it processes each file...\n")

    await translate_all_files()

    print("\nTranslation complete!")

if __name__ == "__main__":
    asyncio.run(main())
