# Progress Tracking & Personalization Features Guide

This document explains how the progress tracking and personalization features work in the Physical AI & Humanoid Robotics platform.

## Progress Tracking System

### Overview
Users can track their learning progress through the platform. A "Mark as Complete" button appears at the bottom of every documentation page, allowing users to mark chapters as completed. This progress is then reflected in the dashboard.

### Components

#### 1. Progress Button (`frontend/src/components/buttons/ProgressButton.tsx`)
- **Location**: Bottom of every documentation page
- **States**:
  - "Mark as Complete" - Initial state
  - "Updating..." - While saving progress
  - "Completed" - After successful completion
  - "Sign in to track progress" - For unauthenticated users

#### 2. Database Model (`backend/src/db/models.py`)
- **Table**: `user_progress`
- **Fields**:
  - `user_id`: Links to the authenticated user
  - `chapter_id`: Unique identifier for the chapter (e.g., "module-1/week-1-introduction/1-1-what-is-ros2")
  - `completion_percentage`: 0-100 (currently set to 100 when marked complete)
  - `time_spent_seconds`: Time tracking (future enhancement)
  - `exercises_completed`: Array of completed exercises
  - `quiz_scores`: Quiz performance tracking

#### 3. API Endpoints (`backend/src/api/progress.py`)
- **POST /api/progress**: Update chapter progress
- **GET /api/progress**: Retrieve all user progress records

#### 4. Dashboard (`frontend/src/pages/dashboard.tsx`)
- **Overall Progress**: Shows percentage of completed chapters out of total (16 chapters)
- **Module Progress**: Calculates completed weeks per module
  - Module 1 (ROS 2 Fundamentals): 4 weeks
  - Module 2 (Robot Simulation): 4 weeks
  - Module 3 (NVIDIA Isaac Sim): 4 weeks
  - Module 4 (VLA Models): 4 weeks
- **Dynamic Updates**: Progress bars update in real-time based on API data

### How It Works

1. User reads a chapter and scrolls to the bottom
2. User clicks "Mark as Complete" button
3. Progress is saved to database via POST request to `/api/progress`
4. Button changes to "Completed" state
5. User visits dashboard to see updated progress percentages
6. Dashboard fetches all progress via GET request to `/api/progress`
7. Calculations determine:
   - Completed chapters (completion_percentage >= 100)
   - Completed weeks per module
   - Overall progress percentage

## Personalization System

### Overview
The platform provides content personalization based on user experience level. Users can select their level (Beginner, Intermediate, Advanced, or Show All) and content is filtered accordingly.

### Experience Levels & Content Mapping

#### Difficulty Levels
All documentation pages have a `difficulty` frontmatter field:
- **beginner**: Module 1 (ROS 2 Fundamentals)
- **intermediate**: Module 2 (Robot Simulation), Module 3 (NVIDIA Isaac Sim)
- **advanced**: Module 4 (VLA Models)

#### Filtering Logic (`frontend/src/contexts/PersonalizationContext.tsx`)

**Beginner Level**:
- Shows ONLY beginner content
- Hides intermediate and advanced content
- Visible: Module 1 only

**Intermediate Level**:
- Shows beginner + intermediate content
- Hides advanced content
- Visible: Module 1, Module 2, Module 3

**Advanced Level**:
- Shows ALL content (beginner + intermediate + advanced)
- Visible: All modules (1, 2, 3, 4)

**Show All**:
- Shows ALL content regardless of difficulty
- Visible: All modules

### Components

#### 1. Personalization Context (`frontend/src/contexts/PersonalizationContext.tsx`)
- **State Management**: Stores user's experience level and personalization settings
- **shouldShowContent()**: Function that determines if content should be visible
- **Integration**: Connected to user profile (hardware_profile.experience_level)

#### 2. Personalization Panel (`frontend/src/components/common/PersonalizationPanel.tsx`)
- **Location**: Bottom-right corner of the page (floating button)
- **Features**:
  - Toggle personalization on/off
  - Select experience level
  - View current settings
  - Shows learning goals

#### 3. DocItem Layout Wrapper (`frontend/src/theme/DocItem/Layout/index.tsx`)
- **Content Filtering**: Checks page difficulty against user's level
- **Filtered Message**: Shows helpful message when content is filtered:
  ```
  🔒 Content Filtered
  This [difficulty] level content is filtered based on your current experience level ([user-level]).
  You can adjust your content preferences using the Content Level button in the bottom right corner.
  ```

#### 4. Difficulty Badge (`frontend/src/components/common/DifficultyBadge.tsx`)
- **Location**: Top of each documentation page
- **Visual Indicators**:
  - 🌱 Beginner (green)
  - 🚀 Intermediate (blue)
  - ⚡ Advanced (purple)

### How It Works

1. User signs in and completes profile questionnaire
2. Experience level is saved to `user.hardware_profile.experience_level`
3. User navigates to documentation
4. PersonalizationContext loads user's experience level
5. For each page:
   - DocItem/Layout reads difficulty from frontmatter
   - Calls `shouldShowContent(difficulty)` to check if page should be visible
   - If visible: Shows content normally
   - If filtered: Shows "Content Filtered" message
6. User can change level anytime via PersonalizationPanel (bottom-right button)
7. Content visibility updates immediately when level changes

### Testing Personalization

**Test Case 1: Beginner User**
- Set level to "Beginner"
- Navigate to Module 1 pages → ✓ Should be visible
- Navigate to Module 2/3 pages → ✗ Should show "Content Filtered"
- Navigate to Module 4 pages → ✗ Should show "Content Filtered"

**Test Case 2: Intermediate User**
- Set level to "Intermediate"
- Navigate to Module 1 pages → ✓ Should be visible
- Navigate to Module 2/3 pages → ✓ Should be visible
- Navigate to Module 4 pages → ✗ Should show "Content Filtered"

**Test Case 3: Advanced User**
- Set level to "Advanced"
- Navigate to any module → ✓ All content should be visible

**Test Case 4: Show All**
- Set level to "Show All"
- Navigate to any module → ✓ All content should be visible

## Key Files

### Frontend
- `frontend/src/components/buttons/ProgressButton.tsx` - Progress tracking button
- `frontend/src/components/common/PersonalizationPanel.tsx` - Personalization UI
- `frontend/src/contexts/PersonalizationContext.tsx` - Personalization logic
- `frontend/src/contexts/AuthContext.tsx` - Authentication state
- `frontend/src/pages/dashboard.tsx` - Progress dashboard
- `frontend/src/theme/DocItem/Content/index.tsx` - Content renderer with progress button
- `frontend/src/theme/DocItem/Layout/index.tsx` - Content filtering wrapper
- `frontend/src/lib/api.ts` - API client functions

### Backend
- `backend/src/api/progress.py` - Progress API endpoints
- `backend/src/db/models.py` - Database models (UserProgress)
- `backend/src/main.py` - App initialization

### Documentation
- All `.md` files in `frontend/docs/` have `difficulty` frontmatter
- 38 files processed with appropriate difficulty levels

## Running the Application

### Start Backend
```bash
cd backend
python -m uvicorn src.main:app --reload
```

### Start Frontend
```bash
cd frontend
npm start
# OR for production build
npm run build && npm run serve
```

### Test Flow
1. Sign up / Sign in
2. Complete profile questionnaire (select experience level)
3. Navigate to documentation pages
4. Verify content filtering based on level
5. Click "Mark as Complete" on a chapter
6. Visit dashboard to see progress updates
7. Change experience level via PersonalizationPanel
8. Verify content filtering updates immediately

## Future Enhancements

- Track time spent on each chapter
- Exercise completion tracking
- Quiz integration
- Progress certificates
- Sidebar filtering (hide filtered content from navigation)
- Progress sync across devices
- Weekly/monthly progress reports
- Achievement badges
