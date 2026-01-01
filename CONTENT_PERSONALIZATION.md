# Content Personalization System

## Overview

The platform now includes a comprehensive content personalization system that filters and displays educational content based on the user's experience level selected during signup.

## Features

### 1. User Experience Levels

Users select their experience level during signup:
- **🌱 Beginner**: New to robotics and AI - sees only beginner-level content
- **🚀 Intermediate**: Some programming experience - sees beginner and intermediate content
- **⚡ Advanced**: Strong technical background - sees all content
- **🌐 Show All**: View all content regardless of difficulty

### 2. Content Filtering

Content is automatically filtered based on:
- User's selected experience level
- Each document's difficulty metadata
- Personalization toggle (can be turned on/off)

**Filtering Rules:**
- **Beginners**: Only see `beginner` level content
- **Intermediate**: See `beginner` and `intermediate` content
- **Advanced**: See all content (beginner, intermediate, advanced)
- **Show All**: See everything (no filtering)

### 3. Visual Indicators

Each document displays a difficulty badge at the top:
- 🌱 **Beginner** (Green) - No prior knowledge required
- 🚀 **Intermediate** (Blue) - Some programming experience helpful
- ⚡ **Advanced** (Purple) - Strong technical background recommended

### 4. Personalization Control Panel

Users can adjust their settings anytime using the **Content Level** button (bottom right):
- Toggle personalization on/off
- Change experience level
- View learning goals
- See current hardware profile

## Implementation Details

### Components Created

1. **PersonalizationContext** (`frontend/src/contexts/PersonalizationContext.tsx`)
   - Manages user experience level
   - Provides content filtering logic
   - Tracks learning goals

2. **PersonalizationPanel** (`frontend/src/components/common/PersonalizationPanel.tsx`)
   - Floating control panel for user settings
   - Experience level selector
   - Personalization toggle

3. **DifficultyBadge** (`frontend/src/components/common/DifficultyBadge.tsx`)
   - Visual indicator for content difficulty
   - Three difficulty levels with icons
   - Responsive design

4. **DocItem Layout Wrapper** (`frontend/src/theme/DocItem/Layout/index.tsx`)
   - Shows difficulty badges on documents
   - Filters content based on user level
   - Displays helpful messages for filtered content

### Backend Integration

**Profile Data Storage:**
- Experience level saved to `user.hardware_profile.experience_level`
- Learning goals saved to `user.learning_goals`
- Hardware info saved to `user.hardware_profile.robotics_kit` and `gpu`

**API Endpoints Used:**
- `POST /api/auth/signup` - Create user account
- `PATCH /api/users/me` - Update user profile
- `GET /api/auth/me` - Get current user profile

### Adding Difficulty to Content

Add difficulty to any markdown file's frontmatter:

```markdown
---
sidebar_position: 1
difficulty: beginner  # or 'intermediate' or 'advanced'
---

# Your Document Title
```

**Examples:**
- `1-1-what-is-ros2.md` → `difficulty: beginner`
- `2-2-working-with-topics.md` → `difficulty: intermediate`
- `2-3-topic-design-patterns.md` → `difficulty: advanced`

## User Flow

### 1. Signup Flow (Updated)

```
User fills signup form
↓
Account created (backend)
↓
Profile questionnaire appears
↓
User selects:
  - Hardware access
  - Experience level ✨ (Personalization key)
  - Learning goals
↓
User logs in
↓
Profile data saved to backend ✨
↓
Redirect to dashboard
```

### 2. Content Viewing

```
User navigates to documentation
↓
PersonalizationContext checks user's level
↓
DocItem checks document difficulty
↓
Content filtered based on rules
↓
IF visible: Show document with difficulty badge
IF hidden: Show "Content Filtered" message
```

### 3. Adjusting Preferences

```
User clicks "Content Level" button (bottom right)
↓
Personalization panel opens
↓
User can:
  - Toggle personalization on/off
  - Change experience level
  - View learning goals
  - See current profile
↓
Changes apply immediately
```

## Testing the Feature

### Test Scenario 1: New User Signup (Beginner)

1. Go to `/signup`
2. Create account with email/password
3. **In profile questionnaire:**
   - Select "No Hardware" or any option
   - **Select "Beginner" for experience level** ✨
   - Select at least one learning goal
4. Complete signup
5. Navigate to documentation
6. **Expected:**
   - Only see documents marked `difficulty: beginner`
   - Documents with `intermediate` or `advanced` show "Content Filtered" message
   - See 🌱 Beginner badges on visible documents

### Test Scenario 2: Adjusting Experience Level

1. Log in as existing user
2. Click **"Content Level"** button (bottom right)
3. **Change experience level** to "Intermediate"
4. Navigate through documentation
5. **Expected:**
   - Now see both `beginner` and `intermediate` content
   - Advanced content still filtered
   - Badges show correct difficulty levels

### Test Scenario 3: View All Content

1. Log in as any user
2. Click **"Content Level"** button
3. Select **"Show All"**
4. Navigate through documentation
5. **Expected:**
   - See all content regardless of difficulty
   - All difficulty badges visible
   - No content filtering

### Test Scenario 4: Disable Personalization

1. Log in as any user
2. Click **"Content Level"** button
3. **Uncheck "Enable personalized content filtering"**
4. **Expected:**
   - All content visible
   - Difficulty badges still shown
   - No filtering applied

## Console Logs

Watch for these logs during testing:

```
[PERSONALIZATION] User experience level: beginner
[PERSONALIZATION] Experience level changed to: intermediate
[PERSONALIZATION] Personalization enabled
[PERSONALIZATION] Personalization disabled
[SIGNUP FLOW] Profile saved successfully
```

## Styling and UX

### Difficulty Badge Colors

- **Beginner (Green)**: Approachable, beginner-friendly
  - Light mode: `#22c55e` border, `rgba(34, 197, 94, 0.1)` background
  - Dark mode: `#4ade80` text

- **Intermediate (Blue)**: Progress, learning
  - Light mode: `#3b82f6` border, `rgba(59, 130, 246, 0.1)` background
  - Dark mode: `#60a5fa` text

- **Advanced (Purple)**: Expert, sophisticated
  - Light mode: `#a855f7` border, `rgba(168, 85, 247, 0.1)` background
  - Dark mode: `#c084fc` text

### Responsive Design

- **Desktop**: Full personalization panel with all options
- **Tablet**: Compact panel, stacked layout
- **Mobile**:
  - Button label hidden (only icon)
  - Panel width: `calc(100vw - 20px)`
  - Simplified layout

## Future Enhancements

Potential improvements:
1. **Learning Path Recommendations** - Suggest next content based on progress
2. **Progress Tracking** - Track completion by difficulty level
3. **Adaptive Difficulty** - Automatically adjust based on quiz scores
4. **Custom Filters** - Filter by learning goals, hardware, etc.
5. **Content Recommendations** - AI-powered content suggestions
6. **Difficulty Progression** - Visual learning path showing progression

## Troubleshooting

### Issue: Personalization panel not appearing
**Solution:** Ensure user is logged in - panel only shows for authenticated users

### Issue: All content showing regardless of level
**Solution:** Check if personalization is enabled in the panel

### Issue: Difficulty badges not showing
**Solution:** Add `difficulty` to document frontmatter

### Issue: Profile data not saving
**Solution:** Check browser console for API errors - verify backend is running

### Issue: Content filtered incorrectly
**Solution:** Verify difficulty value is one of: `beginner`, `intermediate`, `advanced`

## Technical Notes

- Uses React Context API for state management
- LocalStorage used for preference persistence
- Docusaurus theme swizzling for DocItem customization
- Gradients and animations for modern UI
- Fully responsive and accessible
- Dark mode support throughout

## Files Changed/Created

### New Files:
- `frontend/src/contexts/PersonalizationContext.tsx`
- `frontend/src/components/common/PersonalizationPanel.tsx`
- `frontend/src/components/common/PersonalizationPanel.module.css`
- `frontend/src/components/common/DifficultyBadge.tsx`
- `frontend/src/components/common/DifficultyBadge.module.css`
- `frontend/src/theme/DocItem/Layout/index.tsx`
- `frontend/src/theme/DocItem/Layout/styles.module.css`

### Modified Files:
- `frontend/src/theme/Root.tsx` - Added PersonalizationProvider and Panel
- `frontend/src/pages/signup.tsx` - Save profile data after login
- Sample markdown files - Added difficulty metadata

This personalization system provides a tailored learning experience that grows with the user's knowledge and skills!
