# Better Auth - Complete Authentication System

## Overview

Your application now has a comprehensive authentication system with the following features:

### ✅ Implemented Features

1. **Email & Password Authentication**
   - Secure password hashing with bcrypt
   - JWT-based authentication
   - Access & refresh tokens
   - Session management

2. **Email Verification**
   - Automatic email verification on signup
   - Resend verification option
   - 24-hour token expiry

3. **Password Reset**
   - Request password reset link via email
   - Secure token-based reset
   - 1-hour token expiry

4. **Session Management**
   - Persistent sessions with refresh tokens
   - "Remember me" functionality (7 or 30 days)
   - Device tracking (user agent, IP address)
   - Session revocation on logout

5. **Account Security**
   - Failed login tracking
   - Account lockout after 5 failed attempts (30 min)
   - Automatic unlock after timeout

6. **Two-Factor Authentication (2FA)**
   - TOTP-based 2FA
   - QR code generation for authenticator apps
   - Enable/disable 2FA with verification

7. **OAuth Support (Framework Ready)**
   - OAuth provider fields in database
   - Ready for Google, GitHub integration

## API Endpoints

### Authentication

#### 1. Sign Up
```http
POST /api/auth/signup
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "SecurePassword123!"
}
```

**Response:**
```json
{
  "id": "uuid",
  "email": "user@example.com",
  "email_verified": false,
  "two_factor_enabled": false,
  "is_active": true
}
```

#### 2. Login
```http
POST /api/auth/login
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "SecurePassword123!",
  "remember_me": false
}
```

**Response:**
```json
{
  "access_token": "eyJ0eXAiOiJKV1QiLCJhbGc...",
  "refresh_token": "random_secure_token",
  "token_type": "bearer",
  "requires_2fa": false
}
```

#### 3. Refresh Access Token
```http
POST /api/auth/refresh
Content-Type: application/json

{
  "refresh_token": "your_refresh_token"
}
```

**Response:**
```json
{
  "access_token": "new_access_token",
  "token_type": "bearer"
}
```

#### 4. Get Current User
```http
GET /api/auth/me
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "id": "uuid",
  "email": "user@example.com",
  "email_verified": true,
  "two_factor_enabled": false,
  "hardware_profile": {},
  "learning_goals": [],
  "is_active": true
}
```

#### 5. Logout
```http
POST /api/auth/logout
Content-Type: application/json

{
  "refresh_token": "your_refresh_token"
}
```

### Email Verification

#### 6. Verify Email
```http
POST /api/auth/verify-email
Content-Type: application/json

{
  "token": "verification_token_from_email"
}
```

#### 7. Resend Verification Email
```http
POST /api/auth/resend-verification
Content-Type: application/json

{
  "email": "user@example.com"
}
```

### Password Reset

#### 8. Request Password Reset
```http
POST /api/auth/request-password-reset
Content-Type: application/json

{
  "email": "user@example.com"
}
```

#### 9. Reset Password
```http
POST /api/auth/reset-password
Content-Type: application/json

{
  "token": "reset_token_from_email",
  "new_password": "NewSecurePassword123!"
}
```

### Two-Factor Authentication

#### 10. Enable 2FA
```http
POST /api/auth/enable-2fa
Authorization: Bearer {access_token}
```

**Response:**
```json
{
  "secret": "BASE32_SECRET",
  "qr_code_url": "otpauth://totp/Physical%20AI%20Platform:user@example.com?secret=SECRET&issuer=Physical%20AI%20Platform"
}
```

#### 11. Verify 2FA Code
```http
POST /api/auth/verify-2fa
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "code": "123456"
}
```

#### 12. Disable 2FA
```http
POST /api/auth/disable-2fa
Authorization: Bearer {access_token}
Content-Type: application/json

{
  "code": "123456"
}
```

## Frontend Integration

### Using AuthContext

The frontend already has an `AuthContext` that provides:

```typescript
const {
  user,                    // Current user or null
  isAuthenticated,         // Boolean
  isLoading,              // Boolean
  login,                  // (email, password) => Promise
  signup,                 // (email, password) => Promise
  logout,                 // () => void
  refreshUser,            // () => Promise
  requestPasswordReset,   // (email) => Promise
  // ... more methods
} = useAuth();
```

### Example Usage

```typescript
import { useAuth } from '../contexts/AuthContext';

function LoginPage() {
  const { login, isLoading } = useAuth();

  const handleLogin = async (email: string, password: string) => {
    const result = await login(email, password);

    if (result.success) {
      // Redirect to dashboard
      navigate('/dashboard');
    } else {
      // Show error
      setError(result.error);
    }
  };

  // ... rest of component
}
```

## Email Configuration

### For Development

Emails are printed to the console. You'll see output like:

```
============================================================
📧 EMAIL (Development Mode)
To: user@example.com
Subject: Verify your email address

<html content>
============================================================
```

### For Production

Set these environment variables in `backend/.env`:

```env
# Email Configuration
SMTP_HOST=smtp.gmail.com
SMTP_PORT=587
SMTP_USER=your-email@gmail.com
SMTP_PASSWORD=your-app-password
FROM_EMAIL=noreply@yourapp.com
```

For Gmail:
1. Enable 2-Step Verification
2. Generate an App Password
3. Use the app password as `SMTP_PASSWORD`

## Security Features

### Password Requirements
Strong password validation is enforced to prevent weak or dummy passwords:

**Requirements:**
- Minimum 8 characters, maximum 128 characters
- At least one uppercase letter (A-Z)
- At least one lowercase letter (a-z)
- At least one number (0-9)
- At least one special character (!@#$%^&* etc.)
- Cannot be a common weak password (password123, test123, admin123, etc.)
- Cannot contain the same character repeated 4+ times (e.g., aaaa)

**Example Valid Password:** `SecureP@ss123`

**Example Invalid Passwords:**
- `password123` - Too common
- `Test123` - Missing special character
- `test@123` - Missing uppercase letter
- `TEST@123` - Missing lowercase letter
- `TestTest@` - Missing number
- `Tt@1234` - Less than 8 characters

### Email Validation
Email addresses are validated to ensure quality:

**Requirements:**
- Valid email format (user@domain.com)
- No temporary/disposable email addresses
- No obviously fake emails (test@, admin@, dummy@)
- Valid domain format

**Blocked Disposable Domains:**
- tempmail.com, throwaway.email, guerrillamail.com
- mailinator.com, 10minutemail.com, trashmail.com
- And other temporary email services

### Password Hashing
- Uses bcrypt with automatic salt generation
- Password never stored in plaintext

### JWT Tokens
- Access tokens expire in 30 minutes
- Refresh tokens expire in 7-30 days (based on "remember me")
- Tokens signed with `BETTER_AUTH_SECRET`

### Account Protection
- Failed login tracking
- Automatic account lockout after 5 failed attempts
- 30-minute lockout duration
- Automatic unlock after timeout

### Session Security
- Device and IP tracking
- Session revocation on logout
- Refresh token rotation (security best practice)

## Database Schema

### Users Table (Enhanced)
```sql
- id: UUID primary key
- email: unique, indexed
- password_hash: bcrypt hash
- email_verified: boolean
- verification_token: nullable string
- verification_token_expires: nullable timestamp
- reset_token: nullable string
- reset_token_expires: nullable timestamp
- oauth_provider: nullable string
- oauth_id: nullable string
- two_factor_enabled: boolean
- two_factor_secret: nullable string
- failed_login_attempts: integer
- account_locked_until: nullable timestamp
- created_at, last_login, is_active...
```

### Sessions Table (New)
```sql
- id: UUID primary key
- user_id: foreign key to users
- refresh_token: unique, indexed
- device_info: JSON
- ip_address: string
- created_at: timestamp
- expires_at: timestamp
- last_used: timestamp
```

## Testing the Auth Flow

### Testing Validation

You can test the password and email validation using the provided test script:

```bash
cd backend
python test_auth_validation.py
```

This will test:
- Invalid passwords (weak, short, missing requirements)
- Invalid emails (disposable, dummy accounts)
- Valid credentials

### Manual Testing

### 1. Sign Up with Valid Credentials
```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"user@gmail.com","password":"SecureP@ss123"}'
```

**Example with Invalid Password (will fail):**
```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"user@gmail.com","password":"password123"}'
# Response: {"detail": "Password must contain at least one uppercase letter"}
```

**Example with Disposable Email (will fail):**
```bash
curl -X POST http://localhost:8000/api/auth/signup \
  -H "Content-Type: application/json" \
  -d '{"email":"user@tempmail.com","password":"SecureP@ss123"}'
# Response: {"detail": "Temporary or disposable email addresses are not allowed"}
```

### 2. Check Console for Verification Email
Look for the email in your backend console output.

### 3. Verify Email
```bash
curl -X POST http://localhost:8000/api/auth/verify-email \
  -H "Content-Type: application/json" \
  -d '{"token":"TOKEN_FROM_EMAIL"}'
```

### 4. Login
```bash
curl -X POST http://localhost:8000/api/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email":"test@example.com","password":"Test123!"}'
```

### 5. Access Protected Route
```bash
curl http://localhost:8000/api/auth/me \
  -H "Authorization: Bearer YOUR_ACCESS_TOKEN"
```

## Next Steps

1. **Add OAuth Providers**
   - Implement Google OAuth
   - Implement GitHub OAuth
   - See `backend/src/api/auth_oauth.py` (to be created)

2. **Email Templates**
   - Customize email templates in `backend/src/services/email.py`
   - Add branding and styling

3. **Frontend UI**
   - Create password reset pages
   - Add 2FA setup UI
   - Implement session management UI

4. **Rate Limiting**
   - Add rate limiting to login endpoint
   - Prevent brute force attacks

5. **Audit Logging**
   - Log authentication events
   - Track security-relevant actions

## Troubleshooting

### Emails Not Sending
- Check SMTP credentials in `.env`
- For Gmail, ensure app passwords are enabled
- In development, check console output

### Token Expired Errors
- Access tokens expire after 30 minutes
- Use refresh token to get new access token
- Implement automatic token refresh in frontend

### Account Locked
- Wait 30 minutes for automatic unlock
- Or manually update `account_locked_until` in database

### 2FA Issues
- Ensure system time is synchronized
- TOTP codes are time-based
- Use apps like Google Authenticator or Authy

## Support

For issues or questions:
1. Check backend logs: `backend/logs/`
2. Check database: `backend/app.db`
3. Review API responses for error details

---

**🎉 Your authentication system is now fully implemented and ready to use!**
