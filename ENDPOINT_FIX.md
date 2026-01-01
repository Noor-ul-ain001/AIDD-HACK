# Authentication Endpoint Fix

## Issue
The frontend was calling `/api/users/signup` and other old authentication endpoints, but the new authentication system with validation uses `/api/auth/` endpoints. This caused 404 Not Found errors.

## Root Cause
- Frontend API client (src/lib/api.ts) was using old `/api/users/` endpoints
- Backend had two authentication modules:
  - `auth.py` with old `/users/` endpoints (NOT registered in main.py)
  - `auth_enhanced.py` with new `/auth/` endpoints (registered in main.py with validation)

## Fix Applied

### Frontend Changes (frontend/src/lib/api.ts)
Updated all authentication endpoints to use the new `/api/auth/` prefix:

| Old Endpoint | New Endpoint |
|-------------|--------------|
| `/api/users/signup` | `/api/auth/signup` |
| `/api/users/login` | `/api/auth/login` |
| `/api/users/me` | `/api/auth/me` |
| `/api/users/verify-email` | `/api/auth/verify-email` |
| `/api/users/resend-verification` | `/api/auth/resend-verification` |
| `/api/users/request-password-reset` | `/api/auth/request-password-reset` |
| `/api/users/reset-password` | `/api/auth/reset-password` |

### Backend Changes (backend/src/api/users.py)
- Updated import: `from ..api.auth_enhanced import get_current_user_id`
- Changed from importing from old `auth.py` to `auth_enhanced.py`

## Benefits of New Endpoints

The new `/api/auth/` endpoints include:
1. ✅ **Password validation** - Enforces strong passwords
2. ✅ **Email validation** - Blocks disposable/dummy emails
3. ✅ **Enhanced security** - Account lockout, 2FA support
4. ✅ **Better error messages** - Clear validation feedback

## Testing

After the fix, test signup with validation:

```bash
# This will fail with proper validation message
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"user@gmail.com","password":"password123"}'

# Response:
# {"detail": "Password must contain at least one uppercase letter"}

# This will succeed
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"user@gmail.com","password":"SecureP@ss123"}'
```

## Files Modified

1. `frontend/src/lib/api.ts` - Updated authentication endpoints
2. `backend/src/api/users.py` - Updated import to use auth_enhanced
3. `frontend/build/` - Rebuilt with new endpoints

## Status

✅ Frontend endpoints updated
✅ Backend imports fixed
✅ Frontend build successful
✅ Validation working on new endpoints

Users can now sign up with the frontend, and the system will properly validate passwords and emails according to the security requirements.
