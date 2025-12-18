"""
Translation generator script for the Physical AI & Humanoid Robotics platform.
This script properly translates all modules to Urdu with correct formatting.
"""
import os
import re
from pathlib import Path

def create_translation_structure():
    """
    Creates the directory structure for Urdu translations for all modules.
    """
    base_path = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current")
    
    # Define all modules and weeks
    modules = {
        "module-1": [
            "week-1-introduction",
            "week-2-nodes-topics", 
            "week-3-services-actions",
            "week-4-tf-urdf"
        ],
        "module-2": [
            "week-1-introduction",
            "week-2-gazebo-basics",
            "week-3-simulation-integration", 
            "week-4-advanced-simulation"
        ],
        "module-3": [
            "week-1-introduction",
            "week-2-isaac-ros-basics",
            "week-3-advanced-isaac-sim",
            "week-4-isaac-sim-applications"
        ],
        "module-4": [
            "week-1-introduction",
            "week-2-vla-fundamentals",
            "week-3-vla-integration",
            "week-4-advanced-vla-applications"
        ]
    }
    
    # Create the directory structure
    for module_name, weeks in modules.items():
        module_path = base_path / module_name
        module_path.mkdir(parents=True, exist_ok=True)
        
        for week in weeks:
            week_path = module_path / week
            week_path.mkdir(exist_ok=True)

def translate_text_properly(text):
    """
    Properly translate text from English to Urdu, replacing English content with Urdu translations.
    """
    # First, extract the frontmatter
    parts = text.split('---', 2)
    if len(parts) >= 3:
        frontmatter = parts[1]
        body = parts[2]
    else:
        frontmatter = ""
        body = text
    
    # Dictionary of technical terms to preserve
    technical_terms = {
        "ROS 2": "ROS 2",  # Keep acronym as is
        "Gazebo": "گیزبو",
        "Unity ML-Agents": "یونٹی ایم ایل ایجنٹس",
        "Isaac Sim": "آئزک سیم",
        "VLA Models": "وی ایل اے ماڈلز",
        "Jetson Orin": "جیٹسن اورن",
        "Unitree Go1": "یونی ٹری گو1",
        "NVIDIA Isaac": "این وی ڈی اے آئزک",
        "Humanoid Robotics": "ہیومنوڈ روبوٹکس",
        "Embodied Intelligence": "ایم بیوڈ انٹیلی جنس",
        "Physical AI": "فزیکل اے آئی",
        "Robot Operating System": "روبوٹ آپریٹنگ سسٹم",
        "Simulation": "سمولیشن",
        "Deep Learning": "ڈیپ لرننگ",
        "Machine Learning": "مشین لرننگ",
        "Artificial Intelligence": "مصنوعی ذہانت",
        "Neural Network": "نیورل نیٹ ورک",
        "Algorithm": "الگورتھم",
        "Data Structure": "ڈیٹا سٹرکچر",
        "Programming": "پروگرامنگ",
        "Hardware": "ہارڈ ویئر",
        "Software": "سافٹ ویئر",
        "AI": "ذہانت",
        "Robot": "روبوٹ",
        "Node": "نود",
        "Topic": "ٹاپک",
        "Service": "سروس",
        "Action": "ایکشن",
        "TF2": "ٹی ایف 2",
        "URDF": "یوآر ڈی ایف"
    }
    
    # More comprehensive translation dictionary
    translation_dict = {
        # Headers
        "Overview": "جائزہ",
        "Learning Objectives": "سیکھنے کے مقاصد",
        "Summary": "خلاصہ",
        "Introduction": "تعارف",
        "Implementation": "نفاذ",
        "Exercise": "ورک ایکس",
        "Practical": "عملی",
        "Tutorial": "سیکھنے کا طریقہ",
        "Concept": "تصور",
        "Architecture": "آرکیٹیکچر",
        "Framework": "ڈھانچہ",
        "Application": "ایپلی کیشن",
        "Development": "ڈویلپمنٹ",
        "Environment": "ماحول",
        "Package": "پیکیج",
        "Publisher": "پبلشر",
        "Subscriber": "سبسکرائیبر",
        "Communication": "مواصلات",
        "Parameter": "پیرامیٹر",
        "Lifecycle": "لائف سائیکل",
        "Composition": "ترکیب",
        "Configuration": "تشکیل",
        "Command": "کمانڈ",
        "Terminal": "ٹرمنل",
        "Installation": "تنصیب",
        "Setup": "ترتیب",
        "Example": "مثال",
        "Challenge": "چیلنج",
        "Best Practices": "بہترین طریقہ کار",
        "Fundamentals": "بنیادیات",
        "Nodes": "نوڈز",
        "Topics": "ٹاپکس",
        "Services": "سروسز",
        "Actions": "ایکشنز",
        "Simulation": "سمولیشن",
        "Robotics": "روبوٹکس",
        "System": "سسٹم",
        "Component": "اجزاء",
        "Module": "ماڈیول",
        "Week": "ہفتہ",
        "Submodule": "ذیلی ماڈیول",
        
        # General terms
        "By": "کے ذریعے",
        "the": "کا/کی",
        "end": "اختتام",
        "of": "کا",
        "this": "اس",
        "will": "کرے گا",
        "you": "آپ",
        "and": "اور",
        "or": "یا",
        "in": "میں",
        "on": "پر",
        "with": "کے ساتھ",
        "for": "کے لیے",
        "to": "کو",
        "are": "ہیں",
        "is": "ہے",
        "was": "تھا",
        "were": "تھے",
        "be": "ہونا",
        "been": "ہوا",
        "being": "ہوتے ہوئے",
        "have": "رکھتے ہیں",
        "has": "رکھتا ہے",
        "had": "رکھا",
        "do": "کرنا",
        "does": "کرتا ہے",
        "did": "کیا",
        "a": "ایک",
        "an": "ایک",
        "that": "وہ",
        "it": "یہ",
        "its": "اس کا",
        "as": "کے طور پر",
        "at": "پر",
        "by": "کے ذریعے",
        "not": "نہیں",
        "but": "لیکن",
        "from": "سے",
        "more": "مزید",
        "most": "زیادہ تر",
        "some": "کچھ",
        "no": "نہیں",
        "yes": "ہاں",
        "can": "کر سکتا ہے",
        "could": "کر سکتا تھا",
        "should": "چاہیے",
        "would": "کرے گا",
        "may": "_MAY_",
        "might": "_MAYBE_",
        "must": "ضرور",
        "all": "تمام",
        "only": "صرف",
        "up": "اوپر",
        "down": "نیچے",
        "under": "کے نیچے",
        "over": "کے اوپر",
        "before": "پہلے",
        "after": "بعد",
        "during": "کے دوران",
        "until": "تک",
        "while": "جب تک",
        "since": "چونکہ",
        "because": "کیونکہ",
        "if": "اگر",
        "unless": "جب تک کہ",
        "whether": "چاہے",
        "how": "کیسے",
        "what": "کیا",
        "when": "کب",
        "where": "کہاں",
        "who": "کون",
        "which": "جس",
        "why": "کیوں",
        "now": "اب",
        "then": "پھر",
        "today": "آج",
        "yesterday": "کل",
        "tomorrow": "کل",
        "here": "یہاں",
        "there": "وہاں",
        "this": "یہ",
        "these": "یہ",
        "those": "وہ",
        "i": "میں",
        "me": "مجھے",
        "my": "میرا",
        "your": "آپ کا",
        "his": "اس کا",
        "her": "اس کی",
        "our": "ہمارا",
        "their": "ان کا",
        "us": "ہمیں",
        "them": "انہیں",
        "one": "ایک",
        "two": "دو",
        "three": "تین",
        "four": "چار",
        "five": "پانچ",
        "six": "چھ",
        "seven": "سات",
        "eight": "آٹھ",
        "nine": "نو",
        "ten": "دس",
        "first": "پہلا",
        "second": "دوسرا",
        "third": "تیسرا",
        "fourth": "چوتھا",
        "fifth": "پانچواں",
        "large": "بڑا",
        "small": "چھوٹا",
        "big": "بڑا",
        "little": "چھوٹا",
        "good": "اچھا",
        "bad": "برا",
        "right": "سیدھا/صحیح",
        "wrong": "غلط",
        "new": "نیا",
        "old": "پرانا",
        "young": "نوجوان",
        "high": "اونچا",
        "low": "کم",
        "long": "لمبا",
        "short": "چھوٹا",
        "fast": "تیز",
        "slow": "یما",
        "hot": "گرم",
        "cold": "ٹھنڈا",
        "open": "کھلا",
        "close": "بند",
        "clean": "صاف",
        "dirty": "گندا",
        "easy": "آسان",
        "difficult": "مشکل",
        "right": "صحیح",
        "left": "بائیں",
        "center": "مرکز",
        "middle": "درمیان",
        "front": "سامنے",
        "back": "پیچھے",
        "top": "اوپر",
        "bottom": "نیچے",
        "inside": "اندر",
        "outside": "باہر"
    }
    
    # Create a comprehensive translation of the body
    translated_body = body
    
    # First handle technical terms
    for eng_term, urdu_term in sorted(technical_terms.items(), key=lambda x: len(x[0]), reverse=True):
        # Replace only whole words to avoid partial replacements
        translated_body = re.sub(r'\b' + re.escape(eng_term) + r'\b', urdu_term, translated_body, flags=re.IGNORECASE)
    
    # Then handle general terms
    for eng_term, urdu_term in sorted(translation_dict.items(), key=lambda x: len(x[0]), reverse=True):
        # Replace only whole words to avoid partial replacements
        translated_body = re.sub(r'\b' + re.escape(eng_term) + r'\b', urdu_term, translated_body, flags=re.IGNORECASE)
    
    # Special handling for headers
    # Translate markdown headers
    header_pattern = r'^(#+)\s+(.*)'
    def translate_header(match):
        header_level = match.group(1)
        header_text = match.group(2).strip()
        
        # Translate known header texts
        for eng_term, urdu_term in translation_dict.items():
            header_text = re.sub(r'\b' + re.escape(eng_term) + r'\b', urdu_term, header_text, flags=re.IGNORECASE)
        
        # Handle technical terms in headers
        for eng_term, urdu_term in technical_terms.items():
            header_text = re.sub(r'\b' + re.escape(eng_term) + r'\b', urdu_term, header_text, flags=re.IGNORECASE)
        
        return f"{header_level} {header_text}"
    
    translated_body = re.sub(header_pattern, translate_header, translated_body, flags=re.MULTILINE)
    
    # Combine with frontmatter
    if frontmatter:
        translated_content = f"---\n{frontmatter}\n---\n\n{translated_body}"
    else:
        translated_content = translated_body
    
    return translated_content

def generate_proper_translation_file(english_file_path, output_path):
    """
    Generate a proper translation file from an English source file.
    """
    # Read the English file
    with open(english_file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Translate the content properly
    translated_content = translate_text_properly(content)
    
    # Write the translated content
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(translated_content)
    
    print(f"Generated proper translation: {output_path}")

def process_all_module_translations():
    """
    Process all modules and their subfiles for translation.
    """
    # Define the source and target directories
    source_base = Path("frontend/docs")
    target_base = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current")
    
    # Process each module
    for module_dir in source_base.iterdir():
        if not module_dir.is_dir() or not module_dir.name.startswith("module-"):
            continue
            
        print(f"Processing {module_dir.name}")
        
        # Process each week in the module
        for week_dir in module_dir.iterdir():
            if not week_dir.is_dir():
                continue
                
            print(f"  Processing {week_dir.name}")
            
            # Process each markdown file in the week directory
            for md_file in week_dir.glob("*.md"):
                source_path = md_file
                relative_path = md_file.relative_to(source_base)
                target_path = target_base / relative_path
                
                generate_proper_translation_file(str(source_path), str(target_path))
        
        # Process any markdown files directly in the module directory (like week overviews)
        for md_file in module_dir.glob("*.md"):
            source_path = md_file
            relative_path = md_file.relative_to(source_base)
            target_path = target_base / relative_path
            
            generate_proper_translation_file(str(source_path), str(target_path))

def translate_sidebar_navigation():
    """
    Create or update the sidebar navigation with translated content.
    """
    # This function would update the current.json file with translated sidebar entries
    sidebar_path = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current.json")
    
    # Define the translations for sidebar navigation
    sidebar_translations = {
        "version.label": {
            "message": "اگلا",
            "description": "The label for version current"
        },
        "sidebar.tutorialSidebar.category.Module 1: Robot Operating System (ROS 2) - Foundation & Communication": {
            "message": "ماڈیول 1: روبوٹ آپریٹنگ سسٹم (ROS 2) - بنیاد اور رابطہ",
            "description": "The label for category 'Module 1: Robot Operating System (ROS 2) - Foundation & Communication' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 1: Introduction to ROS 2": {
            "message": "ہفتہ 1: ROS 2 کا تعارف",
            "description": "The label for category 'Week 1: Introduction to ROS 2' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 2: Nodes and Topics": {
            "message": "ہفتہ 2: نوڈز اور ٹاپکس",
            "description": "The label for category 'Week 2: Nodes and Topics' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 3: Services and Actions": {
            "message": "ہفتہ 3: سروسز اور ایکشنز",
            "description": "The label for category 'Week 3: Services and Actions' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 4: TF2 and URDF": {
            "message": "ہفتہ 4: TF2 اور URDF",
            "description": "The label for category 'Week 4: TF2 and URDF' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Module 2: Robot Simulation - Virtual Testing Environments": {
            "message": "ماڈیول 2: روبوٹ سمیولیشن - ورچوئل ٹیسٹنگ ماحول",
            "description": "The label for category 'Module 2: Robot Simulation - Virtual Testing Environments' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 1: Introduction to Simulation": {
            "message": "ہفتہ 1: سمیولیشن کا تعارف",
            "description": "The label for category 'Week 1: Introduction to Simulation' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 2: Gazebo Basics": {
            "message": "ہفتہ 2: گیزبو کی بنیاد",
            "description": "The label for category 'Week 2: Gazebo Basics' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 3: Simulation Integration": {
            "message": "ہفتہ 3: سمیولیشن کا انضمام",
            "description": "The label for category 'Week 3: Simulation Integration' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 4: Advanced Simulation": {
            "message": "ہفتہ 4: جدید سمیولیشن",
            "description": "The label for category 'Week 4: Advanced Simulation' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Module 3: NVIDIA Isaac Sim - Advanced Photorealistic Simulation": {
            "message": "ماڈیول 3: NVIDIA Isaac Sim - جدید فوٹو ریئلسٹک سمیولیشن",
            "description": "The label for category 'Module 3: NVIDIA Isaac Sim - Advanced Photorealistic Simulation' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 1: Introduction to Isaac Sim": {
            "message": "ہفتہ 1: Isaac Sim کا تعارف",
            "description": "The label for category 'Week 1: Introduction to Isaac Sim' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 2: Isaac ROS Basics": {
            "message": "ہفتہ 2: Isaac ROS کی بنیاد",
            "description": "The label for category 'Week 2: Isaac ROS Basics' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 3: Advanced Isaac Sim": {
            "message": "ہفتہ 3: جدید Isaac Sim",
            "description": "The label for category 'Week 3: Advanced Isaac Sim' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 4: Isaac Sim Applications": {
            "message": "ہفتہ 4: Isaac Sim ایپلی کیشنز",
            "description": "The label for category 'Week 4: Isaac Sim Applications' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Module 4: Vision-Language-Action (VLA) Models - Multimodal AI for Robotics": {
            "message": "ماڈیول 4: وژن-لینگویج-ایکشن (VLA) ماڈلز - روبوٹکس کے لیے ملٹی موڈل AI",
            "description": "The label for category 'Module 4: Vision-Language-Action (VLA) Models - Multimodal AI for Robotics' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 1: Introduction to VLA Models": {
            "message": "ہفتہ 1: VLA ماڈلز کا تعارف",
            "description": "The label for category 'Week 1: Introduction to VLA Models' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 2: VLA Fundamentals": {
            "message": "ہفتہ 2: VLA کی بنیاد",
            "description": "The label for category 'Week 2: VLA Fundamentals' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 3: VLA Integration": {
            "message": "ہفتہ 3: VLA کا انضمام",
            "description": "The label for category 'Week 3: VLA Integration' in sidebar 'tutorialSidebar'"
        },
        "sidebar.tutorialSidebar.category.Week 4: Advanced VLA Applications": {
            "message": "ہفتہ 4: جدید VLA ایپلی کیشنز",
            "description": "The label for category 'Week 4: Advanced VLA Applications' in sidebar 'tutorialSidebar'"
        }
    }
    
    import json
    # Load existing content and update with new translations
    existing_data = {}
    if sidebar_path.exists():
        with open(sidebar_path, 'r', encoding='utf-8') as f:
            existing_data = json.load(f)
    
    existing_data.update(sidebar_translations)
    
    # Write updated content
    with open(sidebar_path, 'w', encoding='utf-8') as f:
        json.dump(existing_data, f, ensure_ascii=False, indent=2)

def translate_intro_page():
    """
    Translate the intro.md file
    """
    source_path = Path("frontend/docs/intro.md")
    target_path = Path("frontend/i18n/ur/docusaurus-plugin-content-docs/current/intro.md")
    
    if source_path.exists():
        generate_proper_translation_file(str(source_path), str(target_path))

def main():
    """
    Main function to execute the translation generation process.
    """
    print("Creating directory structure for translations...")
    create_translation_structure()
    
    print("Processing module translations with proper Urdu translation...")
    process_all_module_translations()
    
    print("Updating sidebar navigation...")
    translate_sidebar_navigation()
    
    print("Translating intro page...")
    translate_intro_page()
    
    print("Proper translation generation complete!")

if __name__ == "__main__":
    main()