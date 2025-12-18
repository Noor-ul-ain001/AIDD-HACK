# اردو ترجمہ گائیڈ - Urdu Translation Guide

## مسئلہ کی تشخیص - Problem Identification

The `frontend/i18n/ur/` folder contains **44 markdown files** that are currently in **English** and need complete translation to **pure Urdu** without any English mixing.

### فائلوں کی فہرست - Files List:
- intro.md (1 file)
- Module 1: 16 files
- Module 2: 8 files
- Module 3: 7 files
- Module 4: 7 files
- Overview files: 5 files

**Total: 44 documentation files**

## ترجمہ کے اصول - Translation Guidelines

### ✅ کیا ترجمہ کریں - What to Translate:
1. تمام عنوانات (All headings)
2. تمام پیراگراف (All paragraphs)
3. فہرست کے تمام نکات (All list items)
4. تمام وضاحتیں (All descriptions)

### ❌ کیا ترجمہ نہ کریں - What NOT to Translate:
1. **Code blocks** (``` سے گھرا ہوا کوڈ)
2. **Technical commands** (e.g., `ros2 run`, `colcon build`)
3. **URLs and links**
4. **File paths** (e.g., `/opt/ros/humble`)
5. **Technical terms** in code context (but translate their explanation)

### 🔤 تکنیکی اصطلاحات - Technical Terms Handling:

Keep these in English when used as technical terms:
- ROS 2
- Node, Topic, Service, Action
- Publisher, Subscriber
- Python, C++
- Gazebo, Isaac Sim
- URDF, TF2, DDS

But translate their descriptions in Urdu:
- Node = نوڈ (and then explain: "ایک پروسیس جو کمپیوٹیشن کرتا ہے")
- Topic = ٹاپک (and explain: "ایک مواصلاتی چینل")

## Option 1: Manual Translation (Recommended for Quality)

### مکمل دستی ترجمہ کا عمل:
1. ہر فائل کو کھولیں
2. انگریزی مواد کو سمجھیں
3. خالص اردو میں ترجمہ کریں
4. کوڈ بلاکس کو برقرار رکھیں
5. محفوظ کریں اور اگلی فائل پر جائیں

### Time Estimate:
- Per file: 30-60 minutes (depending on content length)
- Total: 22-44 hours for all 44 files

## Option 2: AI-Assisted Translation

### Using Google Translate API or Similar:
```python
# Example using googletrans library
from googletrans import Translator
translator = Translator()
result = translator.translate(english_text, src='en', dest='ur')
urdu_text = result.text
```

### چیلنجز - Challenges:
- May produce awkward phrasing
- Technical terms might be incorrectly translated
- Requires manual review and correction
- May mix romanized Urdu

## Option 3: Professional Translation Service

### تجویز کردہ خدمات:
1. **Fiverr** - Professional Urdu translators ($50-200 for this volume)
2. **Upwork** - Technical translation specialists
3. **Local translation agencies** in Pakistan

### Requirements when hiring:
- Must be native Urdu speaker
- Should have technical/educational content experience
- Provide sample translation first
- Keep technical terms as specified

## نمونہ ترجمہ - Sample Translation

### ❌ غلط (Incorrect - Mixed Language):
```markdown
# 1.1: What is ROS 2?

یہ submodule provides ایک detailed exploration of Robot Operating System.
```

### ✅ درست (Correct - Pure Urdu):
```markdown
# 1.1: ROS 2 کیا ہے؟

یہ ذیلی ماڈیول روبوٹ آپریٹنگ سسٹم 2 (ROS 2) کی تفصیلی تلاش فراہم کرتا ہے - روبوٹ سافٹ ویئر ایپلیکیشنز تیار کرنے کے لیے اگلی نسل کا فریم ورک۔
```

## Quick Start - فوری آغاز

### اگر آپ خود ترجمہ کر رہے ہیں:

1. **Start with intro.md** (easiest, most important)
2. **Then do Module 1** (foundation content)
3. **Use consistency** - keep a glossary of translated terms
4. **Review each file** - ensure no English mixing

### اگر آپ مدد چاہتے ہیں:

I can help translate files one by one. Just tell me which file to translate next and I'll provide complete pure Urdu translation.

## ترجمہ شدہ اصطلاحات کی فہرست - Translated Terms Glossary

| English | اردو | Context |
|---------|------|---------|
| Overview | خلاصہ | Section heading |
| Learning Objectives | سیکھنے کے مقاصد | Section heading |
| By the end of this module | اس ماڈیول کے اختتام تک | Common phrase |
| You will understand | آپ سمجھیں گے | Common phrase |
| You will learn | آپ سیکھیں گے | Common phrase |
| Prerequisites | پیش شرطیں | Requirements |
| Installation | تنصیب | Setup |
| Configuration | ترتیب | Setup |
| Example | مثال | Code samples |
| Note | نوٹ | Admonitions |
| Important | اہم | Admonitions |
| Warning | انتباہ | Admonitions |
| Summary | خلاصہ | Conclusion |

## اگلے اقدامات - Next Steps

**فوری ترجمہ کے لیے:**
Tell me which files you want me to translate first, and I'll provide complete, pure Urdu translations with no English mixing (except for code blocks and technical commands).

**یا**

Use one of the options above based on your timeline and budget.

---

**نوٹ:** Complete professional translation of all 44 files requires significant time and expertise. The best approach is either:
1. Translate progressively (most important files first)
2. Hire a professional Urdu technical translator
3. Ask me to translate specific files one at a time
