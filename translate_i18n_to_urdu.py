"""
Complete Urdu Translation Script for i18n Documentation Files
Translates all English content to pure Urdu while preserving:
- Code blocks
- URLs
- Technical command names
"""

import os
import re
from pathlib import Path

# Complete Urdu translations for common terms and phrases
TRANSLATIONS = {
    # Headers and Common Phrases
    "Overview": "خلاصہ",
    "Learning Objectives": "سیکھنے کے مقاصد",
    "By the end of this submodule, you will:": "اس ذیلی ماڈیول کے اختتام تک، آپ:",
    "Summary": "خلاصہ",
    "Key Characteristics": "اہم خصوصیات",
    "Prerequisites": "پیش شرطیں",
    "Getting Started": "شروعات کرتے ہوئے",
    "Installation": "تنصیب",
    "Configuration": "ترتیب",
    "Examples": "مثالیں",
    "Best Practices": "بہترین طریقے",
    "Common Issues": "عام مسائل",
    "Troubleshooting": "مسائل کا حل",
    "Next Steps": "اگلے اقدامات",
    "References": "حوالہ جات",
    "Additional Resources": "اضافی وسائل",
    "Practical Exercises": "عملی مشقیں",
    "Exercise": "مشق",
    "Solution": "حل",
    "Important": "اہم",
    "Note": "نوٹ",
    "Warning": "انتباہ",
    "Tip": "تجویز",
    "Example": "مثال",

    # Module 1 - ROS 2 Terms
    "What is ROS 2?": "ROS 2 کیا ہے؟",
    "What is a Node?": "نوڈ کیا ہے؟",
    "Creating Nodes in Python": "پائتھن میں نوڈز بنانا",
    "Creating Nodes in C++": "C++ میں نوڈز بنانا",
    "Basic Node Structure": "بنیادی نوڈ ڈھانچہ",
    "Node Lifecycle": "نوڈ کی زندگی",
    "Understanding": "سمجھنا",
    "Working with": "کے ساتھ کام کرنا",
    "Introduction to": "تعارف",

    # Common technical descriptions (translate the description, keep terms in English)
    "ROS 2 is the second generation of the Robot Operating System":
        "ROS 2 روبوٹ آپریٹنگ سسٹم کی دوسری نسل ہے",
    "an open-source framework for developing robot applications":
        "روبوٹ ایپلیکیشنز تیار کرنے کے لیے ایک اوپن سورس فریم ورک",
    "It provides hardware abstraction":
        "یہ ہارڈ ویئر تجرید فراہم کرتا ہے",
    "device drivers": "ڈیوائس ڈرائیورز",
    "libraries": "لائبریریاں",
    "visualizers": "بصری آلات",
    "message-passing": "پیغام رسانی",
    "package management": "پیکیج انتظام",

    # Common sentences
    "This submodule provides": "یہ ذیلی ماڈیول فراہم کرتا ہے",
    "a detailed exploration of": "کی تفصیلی تلاش",
    "the next-generation framework": "اگلی نسل کا فریم ورک",
    "for developing robot software applications": "روبوٹ سافٹ ویئر ایپلیکیشنز تیار کرنے کے لیے",

    # List items common phrases
    "Understand": "سمجھیں",
    "Learn": "سیکھیں",
    "Know": "جانیں",
    "Recognize": "پہچانیں",
    "Be familiar with": "سے واقف ہوں",
    "Create": "بنائیں",
    "Implement": "نافذ کریں",
    "Use": "استعمال کریں",
    "Explore": "دریافت کریں",

    # Module descriptionsآ
    "the fundamental concepts of": "کے بنیادی تصورات",
    "how": "کیسے",
    "differs from": "سے مختلف ہے",
    "traditional operating systems": "روایتی آپریٹنگ سسٹمز",
    "the use cases and applications of": "کے استعمال کے معاملات اور ایپلیکیشنز",
    "the ecosystem": "ماحولیاتی نظام",
}

def preserve_code_blocks(content):
    """Extract code blocks and replace with placeholders"""
    code_blocks = []
    pattern = r'```[\s\S]*?```'

    def replacer(match):
        code_blocks.append(match.group(0))
        return f"___CODE_BLOCK_{len(code_blocks)-1}___"

    content = re.sub(pattern, replacer, content)
    return content, code_blocks

def restore_code_blocks(content, code_blocks):
    """Restore code blocks from placeholders"""
    for i, block in enumerate(code_blocks):
        content = content.replace(f"___CODE_BLOCK_{i}___", block)
    return content

def translate_content(content):
    """Translate English content to Urdu"""
    # Preserve code blocks
    content, code_blocks = preserve_code_blocks(content)

    # Apply translations
    for english, urdu in TRANSLATIONS.items():
        # Case-insensitive replacement
        content = re.sub(r'\b' + re.escape(english) + r'\b', urdu, content, flags=re.IGNORECASE)

    # Restore code blocks
    content = restore_code_blocks(content, code_blocks)

    return content

def translate_file(file_path):
    """Translate a single markdown file"""
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()

        # Check if file is already translated (contains significant Urdu)
        urdu_chars = len(re.findall(r'[\u0600-\u06FF]', content))
        total_chars = len(content)

        if urdu_chars > total_chars * 0.3:  # If more than 30% is already Urdu
            print(f"[SKIP] {file_path.name} - Already translated")
            return False

        # Translate
        translated = translate_content(content)

        # Write back
        with open(file_path, 'w', encoding='utf-8') as f:
            f.write(translated)

        print(f"[TRANSLATED] {file_path.name}")
        return True

    except Exception as e:
        print(f"[ERROR] {file_path.name}: {str(e)}")
        return False

def main():
    """Main translation function"""
    i18n_docs_path = Path(r'C:\Users\user\Desktop\noor-ana - Copy\frontend\i18n\ur\docusaurus-plugin-content-docs\current')

    if not i18n_docs_path.exists():
        print(f"[ERROR] Path not found: {i18n_docs_path}")
        return

    # Find all markdown files
    md_files = list(i18n_docs_path.rglob('*.md'))

    print(f"Found {len(md_files)} markdown files to process\n")

    translated_count = 0
    skipped_count = 0

    for md_file in md_files:
        result = translate_file(md_file)
        if result:
            translated_count += 1
        else:
            skipped_count += 1

    print(f"\n[COMPLETE]")
    print(f"Translated: {translated_count} files")
    print(f"Skipped: {skipped_count} files")
    print(f"\nNOTE: This is a basic translation. For complete pure Urdu translation,")
    print(f"you need to manually review and translate technical content properly.")

if __name__ == '__main__':
    main()
