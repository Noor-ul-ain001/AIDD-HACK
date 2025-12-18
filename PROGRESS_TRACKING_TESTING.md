# Progress Tracking - Testing & Debugging Guide

## What Was Fixed

### Issues Identified
1. Dashboard wasn't refreshing when returning from documentation pages
2. No visual feedback for data refresh
3. No easy way to manually refresh progress

### Solutions Implemented
1. **Auto-refresh on window focus** - Dashboard automatically refreshes when you switch back to the tab
2. **Manual refresh button** - "🔄 Refresh" button in the dashboard for instant updates
3. **Console logging** - Detailed logs to debug data flow
4. **Error handling** - Better error messages for debugging

## Testing the Progress Tracking System

### Step 1: Start Backend
```bash
cd backend
python -m uvicorn src.main:app --reload
```

You should see:
```
INFO:     Uvicorn running on http://127.0.0.1:8000 (Press CTRL+C to quit)
INFO:     Application startup complete.
```

### Step 2: Start Frontend
```bash
cd frontend
npm start
# OR for production build
npm run build && npm run serve
```

### Step 3: Open Browser Console
1. Press F12 to open Developer Tools
2. Go to "Console" tab
3. Keep this open to see debug logs

### Step 4: Test Flow

#### A. Sign In / Sign Up
1. Navigate to http://localhost:3000
2. Click "Sign In" or "Sign Up"
3. Create account or log in
4. Complete profile questionnaire (select experience level)

#### B. Mark Chapter as Complete
1. Navigate to any doc page, for example:
   - http://localhost:3000/docs/module-1/week-1-introduction/1-1-what-is-ros2
2. Scroll to the bottom of the page
3. You should see: **"📝 Mark as Complete"** button
4. Click the button
5. Watch the console logs:
   ```
   [ProgressButton] Marking chapter as complete: module-1/week-1-introduction/1-1-what-is-ros2
   [ProgressButton] API Response: {data: {...}, status: 200}
   [ProgressButton] Progress updated successfully
   ```
6. Button should change to: **"✅ Completed"**
7. You should see a success message: "Progress updated successfully!"

#### C. Check Dashboard - Method 1 (Navigate)
1. Click "Dashboard" in the navbar
2. Watch the console logs:
   ```
   [Dashboard] Fetching progress data...
   [Dashboard] Progress API response: {data: [...], status: 200}
   [Dashboard] Progress data received: [{chapter_id: "module-1/week-1-introduction/1-1-what-is-ros2", ...}]
   ```
3. Check the "Overall Progress" section:
   - Should show: "1 of 16 chapters completed" (6% or similar)
   - Progress bar should be filled accordingly
4. Check "Your Curriculum" section:
   - Module 1 should show progress if you completed a Module 1 chapter

#### D. Check Dashboard - Method 2 (Refresh Button)
1. While on dashboard, click the **"🔄 Refresh"** button
2. Watch console:
   ```
   [Dashboard] Manual refresh triggered...
   [Dashboard] Progress refreshed manually: [{...}]
   ```
3. Progress should update (if you marked more chapters as complete)

#### E. Test Multiple Chapters
1. Navigate to another doc page:
   - http://localhost:3000/docs/module-1/week-1-introduction/1-2-architecture
2. Click "Mark as Complete"
3. Navigate to another:
   - http://localhost:3000/docs/module-1/week-2-nodes-topics/2-1-understanding-nodes
4. Click "Mark as Complete"
5. Go back to Dashboard
6. Should now show: "3 of 16 chapters completed"

### Step 5: Verify Database

#### Option A: Direct API Call
Open a new terminal and run:
```bash
curl -X GET "http://localhost:8000/api/progress" \
  -H "Authorization: Bearer YOUR_TOKEN_HERE"
```

Replace `YOUR_TOKEN_HERE` with your actual token from localStorage.

#### Option B: Check Browser Network Tab
1. Open DevTools → Network tab
2. Go to Dashboard
3. Find the request to `/api/progress`
4. Click on it and view the "Response" tab
5. You should see an array of progress objects:
```json
[
  {
    "id": "...",
    "user_id": "...",
    "chapter_id": "module-1/week-1-introduction/1-1-what-is-ros2",
    "completion_percentage": 100,
    "time_spent_seconds": 0,
    ...
  }
]
```

## Common Issues & Solutions

### Issue 1: Button Not Showing
**Symptoms**: No progress button at bottom of doc pages

**Solutions**:
1. Make sure you're on a documentation page (not intro or dashboard)
2. Check browser console for errors
3. Verify the build succeeded
4. Try hard refresh (Ctrl+Shift+R)

### Issue 2: Button Shows "Sign in to track progress"
**Symptoms**: Button is disabled and says to sign in

**Solution**: You need to be authenticated
1. Click "Sign In" in navbar
2. Log in or create an account
3. Refresh the page

### Issue 3: Progress Not Updating on Dashboard
**Symptoms**: Clicked "Mark as Complete" but dashboard shows 0%

**Debug Steps**:
1. **Check Console Logs**:
   - Look for `[ProgressButton] API Response:` - should show status: 200
   - Look for `[Dashboard] Progress data received:` - should show your data

2. **Check Network Tab**:
   - Find POST to `/api/progress` - should be 200 OK
   - Find GET to `/api/progress` - should return your data

3. **Try Manual Refresh**:
   - Click "🔄 Refresh" button on dashboard
   - Check console for refresh logs

4. **Check Backend Logs**:
   - Look at your backend terminal
   - Should see POST and GET requests

5. **Verify Chapter ID Format**:
   - Console log should show: `chapter_id: "module-1/week-1-introduction/1-1-what-is-ros2"`
   - Should NOT be just "1-1-what-is-ros2"

### Issue 4: Dashboard Shows Wrong Progress
**Symptoms**: Completed 3 chapters but dashboard shows different number

**Debug**:
1. Check console: `[Dashboard] Progress data received:`
2. Count items in the array
3. Verify each `completion_percentage >= 100`
4. Check if chapter_ids match expected format

### Issue 5: "Failed to update progress" Error
**Symptoms**: Error message appears when clicking button

**Possible Causes**:
1. **Backend not running**
   - Solution: Start backend with `python -m uvicorn src.main:app --reload`

2. **Authentication token expired**
   - Solution: Sign out and sign back in

3. **CORS error**
   - Solution: Check backend CORS settings in `backend/src/main.py`

4. **Database error**
   - Solution: Check backend terminal for error logs

## Debugging with Console Logs

All progress-related actions now have console logs prefixed with tags:

- `[ProgressButton]` - Button component actions
- `[Dashboard]` - Dashboard data fetching
- `[API]` - API calls (if needed)

### Expected Console Output

**When marking a chapter complete:**
```
[ProgressButton] Marking chapter as complete: module-1/week-1-introduction/1-1-what-is-ros2
[ProgressButton] API Response: {data: {id: "...", chapter_id: "...", ...}, status: 200}
[ProgressButton] Progress updated successfully
```

**When viewing dashboard:**
```
[Dashboard] Fetching progress data...
[Dashboard] Progress API response: {data: [...], status: 200}
[Dashboard] Progress data received: [
  {
    id: "...",
    user_id: "...",
    chapter_id: "module-1/week-1-introduction/1-1-what-is-ros2",
    completion_percentage: 100,
    ...
  }
]
```

**When refreshing dashboard:**
```
[Dashboard] Manual refresh triggered...
[Dashboard] Progress refreshed manually: [...]
```

**When switching back to dashboard tab:**
```
[Dashboard] Window focused, refreshing progress...
[Dashboard] Progress refreshed: [...]
```

## Expected Behavior Summary

✅ **What Should Work:**
1. Button appears on every documentation page
2. Clicking button marks chapter as 100% complete
3. Button changes from "Mark as Complete" → "Completed"
4. Dashboard shows updated progress immediately (after refresh/focus)
5. Manual refresh button works
6. Auto-refresh on window focus works
7. Module progress calculates based on completed weeks
8. Overall progress shows completed chapters / total chapters

✅ **Progress Calculation:**
- **Overall Progress**: (Completed Chapters / 16 Total Chapters) × 100
- **Module Progress**: (Completed Weeks / Total Weeks) × 100
  - Module 1: 4 weeks
  - Module 2: 4 weeks
  - Module 3: 4 weeks
  - Module 4: 4 weeks
- **Completed Week**: At least one chapter in that week is marked complete

## Need More Help?

If progress still isn't updating:
1. Share the console logs (copy/paste the [ProgressButton] and [Dashboard] logs)
2. Share the Network tab response from `/api/progress`
3. Share any error messages from backend terminal
4. Verify the chapter_id format in the database
