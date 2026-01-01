# How to Test Translation Feature

This guide shows you how to verify that content translation is working.

## Method 1: Using Mock Translation (No Backend Required) ✅

This is the **easiest way** to test without running the backend API.

### Steps:

1. **Start the dev server:**
   ```bash
   cd frontend
   npm run start
   ```

2. **Open Browser Console** (F12 or right-click → Inspect)

3. **Enable mock mode** - In the browser console, type:
   ```javascript
   window.ENABLE_MOCK_TRANSLATION = true
   ```

4. **Navigate to any documentation page**
   - Example: http://localhost:3000/docs/module-1/week-1-introduction/1-1-what-is-ros2

5. **Open the chat panel** - Click the 🤖 icon in bottom right

6. **Click the Translation tab** - Click the 🌐 icon

7. **Click "اردو" button** - Switch to Urdu

### What You Should See:

**In Browser Console:**
```
🔄 [MOCK] Translating chapter: module-1/week-1-introduction/1-1-what-is-ros2
✅ [MOCK] Translation completed
```

**On the Page:**
- Loading indicator appears: "ترجمہ جاری ہے..." (for 1.5 seconds)
- Content changes to Urdu (right-to-left)
- Mock content appears:
  ```
  یہ ایک ٹیسٹ ترجمہ ہے
  باب کی شناخت: module-1/week-1-introduction/1-1-what-is-ros2
  یہ مواد اردو میں ترجمہ شدہ ہے۔ یہ صرف ایک ٹیسٹ ہے...
  ```
- Text flows from right to left
- Urdu Nastaliq font is applied
- Code blocks remain English (left-to-right)

**To Switch Back:**
- Click "English" button
- Content returns to original English

---

## Method 2: Using Browser DevTools (Advanced)

### Check Translation State:

1. Open **React DevTools** (install extension if needed)
2. Find the `TranslationProvider` component
3. Look at the state/context:
   ```javascript
   {
     isTranslating: false,
     translatedContent: "<div>...</div>",  // Urdu HTML when translated
     currentLanguage: "ur",
     translationError: null
   }
   ```

### Inspect DOM Changes:

1. Open **Elements/Inspector** tab
2. Click Urdu button
3. Watch the content div change:
   - **Before:** `<div class="markdown">` (contains original MDX)
   - **After:** `<div class="markdown urdu-content" dir="rtl">` (contains translated HTML)

---

## Method 3: With Real Backend API

If your backend is running:

1. **Disable mock mode:**
   ```javascript
   window.ENABLE_MOCK_TRANSLATION = false
   // or just refresh the page
   ```

2. **Verify backend is running:**
   ```bash
   # Check if backend is accessible
   curl http://localhost:8000/api/translate -X POST \
     -H "Content-Type: application/json" \
     -d '{"chapter_id":"test","target_language":"ur"}'
   ```

3. **Click Urdu button** and check console:
   ```
   🔄 Translating chapter: module-1/week-1-introduction/1-1-what-is-ros2 to ur
   ✅ Translation completed successfully
   ```

4. **Check Network tab:**
   - Look for POST request to `/api/translate`
   - Check response contains `translatedContent`

---

## Debugging Checklist

### ✅ Translation Button Working?
- [ ] Button changes text from "Learn in Urdu" to "Learn in English"
- [ ] `localStorage.getItem('preferred_language')` shows `"ur"` when clicked
- [ ] Console shows language change: `Language changed to: ur`

### ✅ Translation API Called?
- [ ] Console shows: `🔄 Translating chapter: ...`
- [ ] Network tab shows POST to `/api/translate` (or mock message in console)
- [ ] No error messages in console

### ✅ Content Changes?
- [ ] Loading indicator appears briefly
- [ ] Page content changes to Urdu text
- [ ] Content flows right-to-left (RTL)
- [ ] Urdu font is applied (Noto Nastaliq Urdu)

### ✅ Visual Verification:
Open browser console and type:
```javascript
// Check current language
localStorage.getItem('preferred_language')  // Should be "ur"

// Check translation state
// (Use React DevTools to inspect TranslationProvider context)

// Check if content has RTL class
document.querySelector('.urdu-content')  // Should exist when translated

// Check direction
document.querySelector('.markdown').getAttribute('dir')  // Should be "rtl"
```

---

## Common Issues

### Issue: "Nothing happens when clicking Urdu"
**Solution:**
- Open console to see error messages
- Make sure `window.ENABLE_MOCK_TRANSLATION = true` is set
- Check if TranslationProvider is wrapping the app

### Issue: "Loading never stops"
**Solution:**
- Backend API might be down (if not using mock)
- Check Network tab for failed requests
- Enable mock mode to test UI without backend

### Issue: "Content doesn't change"
**Solution:**
- Check console for `✅ Translation completed`
- Verify `translatedContent` is not null in React DevTools
- Inspect DOM to see if content div updated

### Issue: "Content changes but looks wrong"
**Solution:**
- Check if `urdu.css` is loaded
- Verify `dir="rtl"` attribute is set
- Check if `.urdu-content` class is applied

---

## Quick Test Script

Copy-paste this into browser console for a full test:

```javascript
// Enable mock mode
window.ENABLE_MOCK_TRANSLATION = true;

// Get current state
console.log('Current language:', localStorage.getItem('preferred_language'));

// After clicking Urdu button, check:
setTimeout(() => {
  console.log('Content is RTL?', document.querySelector('.markdown')?.getAttribute('dir') === 'rtl');
  console.log('Urdu class applied?', document.querySelector('.urdu-content') !== null);
  console.log('Translation completed?', document.querySelector('.translated-content') !== null);
}, 3000);
```

---

## Success Indicators

When translation is working correctly, you should see:

1. ✅ Console logs: `🔄 [MOCK] Translating...` → `✅ [MOCK] Translation completed`
2. ✅ Loading spinner with Urdu text
3. ✅ Content switches to Urdu mock text
4. ✅ RTL layout (content flows right-to-left)
5. ✅ Urdu Nastaliq font visible
6. ✅ Switching back to English restores original content

**If all these work, your translation feature is implemented correctly!** 🎉
