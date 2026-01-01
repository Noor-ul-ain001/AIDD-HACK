"""
Enhanced authentication API with email verification, password reset,
OAuth, 2FA, and session management.
"""
from fastapi import APIRouter, Depends, HTTPException, status, Request
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from pydantic import BaseModel, EmailStr
from typing import Optional
from ..core.config import settings
from ..core.database import get_async_db
from ..db.models import User, Session
from ..services.email import email_service
from ..utils.validators import validate_password_strength, validate_email
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy.future import select
from passlib.context import CryptContext
import jwt
import uuid
import secrets
import pyotp
from datetime import datetime, timedelta

router = APIRouter()

pwd_context = CryptContext(schemes=["bcrypt"], deprecated="auto")
security = HTTPBearer()

# ===== Pydantic Models =====

class SignupRequest(BaseModel):
    email: EmailStr
    password: str

class LoginRequest(BaseModel):
    email: EmailStr
    password: str
    remember_me: Optional[bool] = False

class VerifyEmailRequest(BaseModel):
    token: str

class ResendVerificationRequest(BaseModel):
    email: EmailStr

class PasswordResetRequest(BaseModel):
    email: EmailStr

class PasswordResetConfirm(BaseModel):
    token: str
    new_password: str

class Enable2FAResponse(BaseModel):
    secret: str
    qr_code_url: str

class Verify2FARequest(BaseModel):
    code: str

class RefreshTokenRequest(BaseModel):
    refresh_token: str

class UserResponse(BaseModel):
    id: str
    email: str
    email_verified: bool
    two_factor_enabled: bool
    oauth_provider: Optional[str] = None
    hardware_profile: Optional[dict] = None
    learning_goals: Optional[list] = None
    accessibility_prefs: Optional[dict] = None
    is_active: bool
    created_at: Optional[str] = None

# ===== Helper Functions =====

def verify_password(plain_password, hashed_password):
    return pwd_context.verify(plain_password, hashed_password)

def get_password_hash(password):
    return pwd_context.hash(password)

def generate_token():
    """Generate a secure random token."""
    return secrets.token_urlsafe(32)

def create_access_token(user_id: str, expires_delta: timedelta = None) -> str:
    """Create a JWT access token."""
    if expires_delta is None:
        expires_delta = timedelta(minutes=30)

    expire = datetime.utcnow() + expires_delta
    token_data = {
        "sub": user_id,
        "exp": expire,
        "type": "access"
    }
    return jwt.encode(token_data, settings.BETTER_AUTH_SECRET, algorithm="HS256")

def create_refresh_token() -> str:
    """Create a refresh token."""
    return secrets.token_urlsafe(64)

async def get_current_user_id(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> str:
    """Extract and verify JWT token."""
    token = credentials.credentials

    try:
        payload = jwt.decode(token, settings.BETTER_AUTH_SECRET, algorithms=["HS256"])
        user_id: str = payload.get("sub")
        token_type: str = payload.get("type")

        if user_id is None or token_type != "access":
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid authentication credentials"
            )

        return user_id
    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.JWTError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate credentials"
        )

async def check_account_locked(user: User) -> bool:
    """Check if account is locked due to failed login attempts."""
    if user.account_locked_until and user.account_locked_until > datetime.utcnow():
        return True
    return False

async def get_optional_user_id(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(HTTPBearer(auto_error=False))
) -> Optional[str]:
    """Extract user ID from token if present, otherwise return None."""
    if not credentials:
        return None

    try:
        payload = jwt.decode(credentials.credentials, settings.BETTER_AUTH_SECRET, algorithms=["HS256"])
        return payload.get("sub")
    except jwt.JWTError:
        return None

# ===== Auth Routes =====

@router.post("/auth/signup", response_model=UserResponse)
async def signup(
    request: SignupRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """Create a new user account with email verification."""

    # Validate email
    email_valid, email_error = validate_email(request.email)
    if not email_valid:
        raise HTTPException(status_code=400, detail=email_error)

    # Validate password strength
    password_valid, password_error = validate_password_strength(request.password)
    if not password_valid:
        raise HTTPException(status_code=400, detail=password_error)

    # Check if user already exists
    result = await db.execute(select(User).filter(User.email == request.email))
    existing_user = result.scalar_one_or_none()

    if existing_user:
        raise HTTPException(status_code=400, detail="User already exists")

    # Hash the password
    password_hash = get_password_hash(request.password)

    # Generate verification token
    verification_token = generate_token()
    verification_expires = datetime.utcnow() + timedelta(hours=24)

    # Create new user
    user = User(
        id=str(uuid.uuid4()),
        email=request.email,
        password_hash=password_hash,
        email_verified=False,
        verification_token=verification_token,
        verification_token_expires=verification_expires,
        hardware_profile={},
        learning_goals=[],
        accessibility_prefs={},
        is_active=True
    )

    db.add(user)
    await db.commit()
    await db.refresh(user)

    # Send verification email
    await email_service.send_verification_email(user.email, verification_token)

    return UserResponse(
        id=user.id,
        email=user.email,
        email_verified=user.email_verified,
        two_factor_enabled=user.two_factor_enabled,
        oauth_provider=user.oauth_provider,
        hardware_profile=user.hardware_profile,
        learning_goals=user.learning_goals,
        accessibility_prefs=user.accessibility_prefs,
        is_active=user.is_active,
        created_at=user.created_at.isoformat() if user.created_at else None
    )

@router.post("/auth/verify-email")
async def verify_email(
    request: VerifyEmailRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """Verify user email with token."""

    result = await db.execute(
        select(User).filter(User.verification_token == request.token)
    )
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(status_code=400, detail="Invalid verification token")

    if user.verification_token_expires < datetime.utcnow():
        raise HTTPException(status_code=400, detail="Verification token has expired")

    # Mark email as verified
    user.email_verified = True
    user.verification_token = None
    user.verification_token_expires = None

    await db.commit()

    return {"message": "Email verified successfully"}

@router.post("/auth/resend-verification")
async def resend_verification(
    request: Optional[ResendVerificationRequest] = None,
    user_id: Optional[str] = Depends(get_optional_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """Resend email verification link. Can be called by logged-in user or with email."""

    # If user is logged in, use their email
    if user_id:
        result = await db.execute(select(User).filter(User.id == user_id))
        user = result.scalar_one_or_none()
    elif request and request.email:
        result = await db.execute(select(User).filter(User.email == request.email))
        user = result.scalar_one_or_none()
    else:
        raise HTTPException(status_code=400, detail="Email required")

    if not user:
        # Don't reveal if user exists
        return {"message": "If the email exists, a verification link has been sent"}

    if user.email_verified:
        raise HTTPException(status_code=400, detail="Email already verified")

    # Generate new verification token
    verification_token = generate_token()
    verification_expires = datetime.utcnow() + timedelta(hours=24)

    user.verification_token = verification_token
    user.verification_token_expires = verification_expires

    await db.commit()

    # Send verification email
    await email_service.send_verification_email(user.email, verification_token)

    return {"message": "Verification email sent"}

@router.post("/auth/login")
async def login(
    request: LoginRequest,
    req: Request,
    db: AsyncSession = Depends(get_async_db)
):
    """Authenticate user and create session."""

    # Find user
    result = await db.execute(select(User).filter(User.email == request.email))
    user = result.scalar_one_or_none()

    if not user or not verify_password(request.password, user.password_hash):
        # Increment failed login attempts
        if user:
            user.failed_login_attempts += 1

            # Lock account after 5 failed attempts
            if user.failed_login_attempts >= 5:
                user.account_locked_until = datetime.utcnow() + timedelta(minutes=30)
                await db.commit()
                raise HTTPException(
                    status_code=status.HTTP_403_FORBIDDEN,
                    detail="Account locked due to multiple failed login attempts. Try again in 30 minutes."
                )

            await db.commit()

        raise HTTPException(status_code=400, detail="Invalid credentials")

    # Check if account is locked
    if await check_account_locked(user):
        raise HTTPException(
            status_code=status.HTTP_403_FORBIDDEN,
            detail="Account is temporarily locked. Please try again later."
        )

    # Reset failed login attempts
    user.failed_login_attempts = 0
    user.last_login = datetime.utcnow()

    # Create tokens
    access_token = create_access_token(user.id)
    refresh_token = create_refresh_token()

    # Create session
    expires_delta = timedelta(days=30) if request.remember_me else timedelta(days=7)
    session = Session(
        id=str(uuid.uuid4()),
        user_id=user.id,
        refresh_token=refresh_token,
        device_info={"user_agent": req.headers.get("user-agent", "unknown")},
        ip_address=req.client.host if req.client else None,
        expires_at=datetime.utcnow() + expires_delta
    )

    db.add(session)
    await db.commit()

    # Prepare the response with information about verification status
    response_data = {
        "access_token": access_token,
        "refresh_token": refresh_token,
        "token_type": "bearer",
        "requires_2fa": user.two_factor_enabled
    }

    # Include email verification status in the response
    if not user.email_verified:
        response_data["email_verified"] = False
        response_data["message"] = "Please verify your email address for full access"

    return response_data

@router.post("/auth/refresh")
async def refresh_token(
    request: RefreshTokenRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """Refresh access token using refresh token."""

    # Find session
    result = await db.execute(
        select(Session).filter(Session.refresh_token == request.refresh_token)
    )
    session = result.scalar_one_or_none()

    if not session:
        raise HTTPException(status_code=401, detail="Invalid refresh token")

    if session.expires_at < datetime.utcnow():
        await db.delete(session)
        await db.commit()
        raise HTTPException(status_code=401, detail="Refresh token has expired")

    # Update last used
    session.last_used = datetime.utcnow()
    await db.commit()

    # Create new access token
    access_token = create_access_token(session.user_id)

    return {
        "access_token": access_token,
        "token_type": "bearer"
    }

@router.post("/auth/request-password-reset")
async def request_password_reset(
    request: PasswordResetRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """Request password reset link."""

    result = await db.execute(select(User).filter(User.email == request.email))
    user = result.scalar_one_or_none()

    # Don't reveal if user exists
    if user:
        # Generate reset token
        reset_token = generate_token()
        reset_expires = datetime.utcnow() + timedelta(hours=1)

        user.reset_token = reset_token
        user.reset_token_expires = reset_expires

        await db.commit()

        # Send reset email
        await email_service.send_password_reset_email(user.email, reset_token)

    return {"message": "If the email exists, a password reset link has been sent"}

@router.post("/auth/reset-password")
async def reset_password(
    request: PasswordResetConfirm,
    db: AsyncSession = Depends(get_async_db)
):
    """Reset password with token."""

    # Validate new password strength
    password_valid, password_error = validate_password_strength(request.new_password)
    if not password_valid:
        raise HTTPException(status_code=400, detail=password_error)

    result = await db.execute(
        select(User).filter(User.reset_token == request.token)
    )
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(status_code=400, detail="Invalid reset token")

    if user.reset_token_expires < datetime.utcnow():
        raise HTTPException(status_code=400, detail="Reset token has expired")

    # Update password
    user.password_hash = get_password_hash(request.new_password)
    user.reset_token = None
    user.reset_token_expires = None
    user.failed_login_attempts = 0
    user.account_locked_until = None

    await db.commit()

    return {"message": "Password reset successfully"}

@router.get("/auth/me", response_model=UserResponse)
async def get_current_user(
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """Get current user profile."""

    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return UserResponse(
        id=user.id,
        email=user.email,
        email_verified=user.email_verified,
        two_factor_enabled=user.two_factor_enabled,
        oauth_provider=user.oauth_provider,
        hardware_profile=user.hardware_profile,
        learning_goals=user.learning_goals,
        accessibility_prefs=user.accessibility_prefs,
        is_active=user.is_active,
        created_at=user.created_at.isoformat() if user.created_at else None
    )

@router.post("/auth/logout")
async def logout(
    refresh_token: str,
    db: AsyncSession = Depends(get_async_db)
):
    """Logout and invalidate session."""

    result = await db.execute(
        select(Session).filter(Session.refresh_token == refresh_token)
    )
    session = result.scalar_one_or_none()

    if session:
        await db.delete(session)
        await db.commit()

    return {"message": "Logged out successfully"}

@router.post("/auth/enable-2fa", response_model=Enable2FAResponse)
async def enable_2fa(
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """Enable two-factor authentication."""

    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    # Generate TOTP secret
    secret = pyotp.random_base32()
    user.two_factor_secret = secret

    await db.commit()

    # Generate QR code URL
    totp = pyotp.TOTP(secret)
    qr_code_url = totp.provisioning_uri(
        name=user.email,
        issuer_name="Physical AI Platform"
    )

    return Enable2FAResponse(
        secret=secret,
        qr_code_url=qr_code_url
    )

@router.post("/auth/verify-2fa")
async def verify_2fa(
    request: Verify2FARequest,
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """Verify 2FA code and enable 2FA."""

    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user or not user.two_factor_secret:
        raise HTTPException(status_code=400, detail="2FA not initialized")

    # Verify code
    totp = pyotp.TOTP(user.two_factor_secret)
    if not totp.verify(request.code):
        raise HTTPException(status_code=400, detail="Invalid 2FA code")

    # Enable 2FA
    user.two_factor_enabled = True
    await db.commit()

    return {"message": "2FA enabled successfully"}

@router.post("/auth/disable-2fa")
async def disable_2fa(
    request: Verify2FARequest,
    user_id: str = Depends(get_current_user_id),
    db: AsyncSession = Depends(get_async_db)
):
    """Disable two-factor authentication."""

    result = await db.execute(select(User).filter(User.id == user_id))
    user = result.scalar_one_or_none()

    if not user or not user.two_factor_enabled:
        raise HTTPException(status_code=400, detail="2FA not enabled")

    # Verify code before disabling
    totp = pyotp.TOTP(user.two_factor_secret)
    if not totp.verify(request.code):
        raise HTTPException(status_code=400, detail="Invalid 2FA code")

    # Disable 2FA
    user.two_factor_enabled = False
    user.two_factor_secret = None
    await db.commit()

    return {"message": "2FA disabled successfully"}
