# Debug Signup Flow - Testing Instructions

## What I've Fixed

I've added comprehensive debugging and improved timing to diagnose and fix the profile questionnaire issue.

### Changes Made:

1. **Extended Timing:**
   - Loading screen: 1500ms (was 800ms)
   - Completion screen before redirect: 3000ms (was 2000ms)

2. **Added Console Logging:**
   - Every step of the signup flow now logs to browser console
   - Each log message is prefixed with `[SIGNUP FLOW]`, `[SIGNUP FORM]`, or `[PROFILE QUESTIONNAIRE]`

3. **Improved Error Handling:**
   - Errors prevent progression to profile questionnaire
   - User stays on signup form if email already exists
   - Clear error messages displayed

## How to Test

### 1. Open Browser Developer Tools
- Press F12 to open dev tools
- Go to the **Console** tab
- Keep it open while testing

### 2. Try to Create an Account

**IMPORTANT:** Watch the console logs carefully. You should see these messages in this order:

```
[SIGNUP FORM] Starting signup process for: your@email.com
[SIGNUP FORM] Calling backend signup API...
[SIGNUP FORM] Backend response: {success: true, data: {...}}
[SIGNUP FORM] Signup successful! Calling parent onSignup callback...
[SIGNUP FORM] Parent callback completed
[SIGNUP FLOW] Signup successful, proceeding to profile questionnaire
[SIGNUP FLOW] User data: your@email.com
[SIGNUP FLOW] Step set to: loading
[SIGNUP FLOW] Showing profile questionnaire
[SIGNUP FLOW] Step set to: profile
```

### 3. What You Should See

**Expected Flow:**
1. Fill out signup form → Click "Sign Up"
2. **Loading screen appears** (for 1.5 seconds)
3. **Profile questionnaire appears** with 3 steps:
   - Step 1: Hardware Access (radio buttons)
   - Step 2: Experience Level (radio buttons)
   - Step 3: Learning Goals (checkboxes - select at least one)
4. Click "Next" to progress through steps
5. Click "Complete Profile" on step 3
6. **Completion message appears** (for 3 seconds)
7. Automatic redirect to dashboard

### 4. If the Issue Still Occurs

**Copy the console logs and send them to me.** The logs will show:

- ✅ When signup API is called
- ✅ Whether signup succeeded or failed
- ✅ When each step transition happens
- ✅ If any step is being skipped
- ✅ If the questionnaire steps are being auto-advanced

## Common Issues and Solutions

### Issue: "Email already registered" error
**Solution:** This is correct behavior. Use a different email address.

### Issue: Profile questionnaire doesn't appear at all
**Check console for:**
- `[SIGNUP FORM] Backend response` - is it showing success: true?
- `[SIGNUP FLOW] Step set to: profile` - is this message appearing?

### Issue: Questionnaire appears then disappears immediately
**Check console for:**
- Any error messages
- Messages appearing out of order
- Multiple rapid calls to `setStep`

### Issue: Questionnaire auto-completes without user action
**Check console for:**
- `[PROFILE QUESTIONNAIRE] Next button clicked` without you clicking
- `[PROFILE QUESTIONNAIRE] Form submitted` without you clicking "Complete Profile"

## Testing Different Scenarios

### Test 1: New Email (Should Work)
1. Use a unique email: `test123@example.com`
2. Create strong password: `Password123!`
3. Watch for profile questionnaire to appear and stay visible
4. Complete all 3 steps manually

### Test 2: Existing Email (Should Fail Gracefully)
1. Try the same email again
2. You should see error: "This email is already registered. Please sign in or use a different email."
3. You should stay on the signup form (NOT proceed to questionnaire)

### Test 3: Weak Password (Should Fail)
1. Use email: `test456@example.com`
2. Use weak password: `12345678`
3. You should see password validation errors
4. You should stay on the signup form

## What to Report Back

Please provide:

1. **What you see on screen:**
   - Does loading screen appear?
   - Does questionnaire appear?
   - How long does it stay visible?

2. **Console logs:**
   - Copy all messages starting with `[SIGNUP`
   - Include any error messages

3. **When does the issue happen:**
   - Immediately after signup form?
   - After loading screen?
   - During questionnaire display?

This information will help me identify exactly where the problem is occurring.

## Current Status

✅ Build successful
✅ Debug logging added
✅ Timing extended
✅ Error handling improved
⏳ Waiting for your test results and console logs
