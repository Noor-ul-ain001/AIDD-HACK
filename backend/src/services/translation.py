import os
import google.generativeai as genai

# Configure Gemini API
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

async def translate_text_with_gemini(text: str, target_language: str = "Urdu") -> str:
    """
    Translates the given text using the Gemini API.
    Technical terms are preserved with explanations, and code blocks are kept as-is.
    """
    model = genai.GenerativeModel('gemini-1.5-flash') # Using flash for translation

    prompt = f"""Translate the following English text to {target_language}.
    When translating, keep technical terms in English and provide a brief explanation in {target_language} in parentheses.
    Preserve any code blocks exactly as they are, without translation or modification.

    English text:
    ---
    {text}
    ---

    {target_language} translation:
    """

    response = await model.generate_content_async(prompt)
    return response.text
