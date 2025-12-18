# Translation Workflow Guide

This guide explains how to use i18next for translating the Docusaurus documentation/course content.

## Overview

The platform uses two separate translation systems:

1. **Docusaurus i18n** - For documentation content (course material markdown files)
2. **react-i18next** - For custom React UI components (chatbot, authentication, etc.)

## Docusaurus i18n (Documentation Translation)

### Configuration

The Docusaurus i18n is configured in `docusaurus.config.ts`:

```typescript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'ur'],
  localeConfigs: {
    en: {
      label: 'English',
      direction: 'ltr',
      htmlLang: 'en-US',
    },
    ur: {
      label: 'اردو',
      direction: 'rtl',
      htmlLang: 'ur-PK',
    },
  },
},
```

### Directory Structure

```
frontend/
├── docs/                          # English documentation (default)
│   └── intro.md
├── i18n/
│   └── ur/                        # Urdu translations
│       ├── code.json              # Theme UI strings
│       ├── docusaurus-theme-classic/
│       │   ├── navbar.json        # Navbar translations
│       │   └── footer.json        # Footer translations
│       └── docusaurus-plugin-content-docs/
│           └── current/
│               ├── intro.md       # Translated intro page
│               └── ...            # Other translated docs
```

### Workflow for Translating Documentation

#### Step 1: Generate Translation Files

When you add a new locale or update the sidebar structure, regenerate translation files:

```bash
cd frontend
npm run write-translations -- --locale ur
```

This creates/updates JSON files in `i18n/ur/` with all translatable strings.

#### Step 2: Translate UI Strings

Edit the generated JSON files to translate UI elements:

**`i18n/ur/docusaurus-theme-classic/navbar.json`:**
```json
{
  "item.label.Curriculum": {
    "message": "نصاب",
    "description": "Navbar item with label Curriculum"
  }
}
```

**`i18n/ur/docusaurus-plugin-content-docs/current.json`:**
```json
{
  "sidebar.tutorialSidebar.category.Week 1: Introduction to ROS 2": {
    "message": "ہفتہ 1: ROS 2 کا تعارف",
    "description": "The label for category..."
  }
}
```

#### Step 3: Translate Documentation Content

Create translated markdown files in `i18n/ur/docusaurus-plugin-content-docs/current/`:

**Directory mapping:**
- `docs/intro.md` → `i18n/ur/docusaurus-plugin-content-docs/current/intro.md`
- `docs/module-1/week-1-introduction/1-1-what-is-ros2.md` → `i18n/ur/docusaurus-plugin-content-docs/current/module-1/week-1-introduction/1-1-what-is-ros2.md`

**Important notes:**
- Keep the same frontmatter (metadata)
- Preserve code blocks in English
- Maintain the same directory structure

#### Step 4: Build and Test

Build the site for all locales:

```bash
npm run build
```

Serve locally to test:

```bash
npm run serve
```

Visit:
- English: `http://localhost:3000/`
- Urdu: `http://localhost:3000/ur/`

## react-i18next (Custom UI Components)

### Configuration

react-i18next is configured in `src/i18n/config.ts` and initialized in `src/theme/Root.tsx` (the Docusaurus root wrapper).

### Directory Structure

```
frontend/
└── src/
    ├── i18n/
    │   ├── config.ts              # i18next configuration
    │   └── locales/
    │       ├── en.json            # English UI strings
    │       └── ur.json            # Urdu UI strings
    └── contexts/
        └── TranslationContext.tsx # Translation provider
```

### Usage in Components

Import the translation hook:

```tsx
import { useFullTranslation } from '@site/src/contexts/TranslationContext';

function MyComponent() {
  const { t, currentLanguage } = useFullTranslation();

  return (
    <div>
      <h1>{t('translation.title')}</h1>
      <button>{t('common.save')}</button>
    </div>
  );
}
```

### Adding New Translation Keys

Edit `src/i18n/locales/en.json` and `src/i18n/locales/ur.json`:

**English (`en.json`):**
```json
{
  "translation": {
    "myFeature": {
      "title": "My Feature",
      "description": "This is a description"
    }
  }
}
```

**Urdu (`ur.json`):**
```json
{
  "translation": {
    "myFeature": {
      "title": "میری خصوصیت",
      "description": "یہ ایک تفصیل ہے"
    }
  }
}
```

## Translation Best Practices

### 1. Code Blocks and Technical Terms
- Keep code blocks in English (LTR)
- Technical terms (ROS 2, URDF, etc.) can remain in English
- Preserve command examples and API references

### 2. RTL (Right-to-Left) Support
- Urdu content automatically gets RTL styling
- Docusaurus handles RTL direction based on `localeConfigs`
- Code blocks remain LTR within RTL content

### 3. File Organization
- Mirror the English docs structure exactly
- Use the same filenames
- Keep frontmatter consistent

### 4. Incremental Translation
You don't need to translate everything at once:
- Start with high-priority pages (intro, getting started)
- Untranslated pages will fall back to English
- The language dropdown still works

### 5. Maintaining Translations
When updating English content:
1. Update the English markdown file
2. Update the corresponding Urdu file
3. Rebuild and test both locales

## Locale URLs

With Docusaurus i18n, each locale has its own URL prefix:

- **English (default)**: `/docs/intro`
- **Urdu**: `/ur/docs/intro`

The locale dropdown in the navbar allows users to switch between languages, and Docusaurus will:
1. Remember the user's preference
2. Redirect to the same page in the selected locale (if available)
3. Fall back to English if the translation doesn't exist

## Testing Translations

### Local Development

Start the development server for a specific locale:

```bash
# English (default)
npm start

# Urdu
npm start -- --locale ur
```

### Build All Locales

```bash
npm run build
```

This builds both English and Urdu versions and places them in:
- `build/` (English)
- `build/ur/` (Urdu)

### Serve Production Build

```bash
npm run serve
```

Then test both locales at:
- `http://localhost:3000/` (English)
- `http://localhost:3000/ur/` (Urdu)

## Dynamic Content Translation (API-based)

For dynamic content that needs to be translated via the backend API:

1. The user clicks "Translate to Urdu" button
2. The `TranslationContext` calls the backend API with the chapter ID
3. The API returns translated content (preserving code blocks)
4. The UI displays the translated content

This is configured in `src/contexts/TranslationContext.tsx`.

## Common Issues

### Issue: "useTranslation: You will need to pass in an i18next instance"
**Error message**: `react-i18next:: useTranslation: You will need to pass in an i18next instance by using initReactI18next`

**Cause**: The i18next configuration isn't being imported/initialized before React components try to use it.

**Solution**: Ensure `src/i18n/config.ts` is imported in `src/theme/Root.tsx`:
```tsx
// At the top of src/theme/Root.tsx
import '../i18n/config';
```

### Issue: Build fails with broken links
**Solution**: Check that all internal links in translated files use the correct locale prefix or relative paths.

### Issue: Sidebar categories not translated
**Solution**: Re-run `npm run write-translations -- --locale ur` and translate entries in `i18n/ur/docusaurus-plugin-content-docs/current.json`.

### Issue: Custom components not translating
**Solution**: Ensure components use the `useFullTranslation()` hook and have entries in `src/i18n/locales/*.json`.

## Resources

- [Docusaurus i18n Documentation](https://docusaurus.io/docs/i18n/introduction)
- [react-i18next Documentation](https://react.i18next.com/)
- [i18next Documentation](https://www.i18next.com/)
