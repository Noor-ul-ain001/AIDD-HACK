from sqlalchemy import Column, Integer, String, DateTime, Boolean, Text, JSON
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()

def generate_uuid():
    return str(uuid.uuid4())

class User(Base):
    __tablename__ = "users"

    id = Column(String, primary_key=True, default=generate_uuid)
    email = Column(String, unique=True, index=True, nullable=False)
    password_hash = Column(String, nullable=True)  # Nullable for OAuth users
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    last_login = Column(DateTime(timezone=True), onupdate=func.now())

    # Email verification
    email_verified = Column(Boolean, default=False)
    verification_token = Column(String, nullable=True)
    verification_token_expires = Column(DateTime(timezone=True), nullable=True)

    # Password reset
    reset_token = Column(String, nullable=True)
    reset_token_expires = Column(DateTime(timezone=True), nullable=True)

    # OAuth
    oauth_provider = Column(String, nullable=True)  # 'google', 'github', etc.
    oauth_id = Column(String, nullable=True)

    # Two-Factor Authentication
    two_factor_enabled = Column(Boolean, default=False)
    two_factor_secret = Column(String, nullable=True)

    # Account security
    failed_login_attempts = Column(Integer, default=0)
    account_locked_until = Column(DateTime(timezone=True), nullable=True)

    # User preferences
    hardware_profile = Column(JSON)  # {gpu, robotics_kit, experience_level}
    learning_goals = Column(JSON)  # ["ROS 2", "Simulation", ...]
    accessibility_prefs = Column(JSON)  # {font_size, reduced_animations, language}
    is_active = Column(Boolean, default=True)


class Session(Base):
    __tablename__ = "sessions"

    id = Column(String, primary_key=True, default=generate_uuid)
    user_id = Column(String, index=True, nullable=False)
    refresh_token = Column(String, unique=True, index=True, nullable=False)
    device_info = Column(JSON, nullable=True)  # {user_agent, browser, os}
    ip_address = Column(String, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    expires_at = Column(DateTime(timezone=True), nullable=False)
    last_used = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())


class UserProgress(Base):
    __tablename__ = "user_progress"

    id = Column(String, primary_key=True, default=generate_uuid)
    user_id = Column(String, index=True, nullable=False)  # Foreign key to User.id
    chapter_id = Column(String, index=True, nullable=False)  # e.g., "module-1/week-1-introduction"
    completion_percentage = Column(Integer, nullable=False)  # 0-100
    last_accessed = Column(DateTime(timezone=True), server_default=func.now())
    time_spent_seconds = Column(Integer, default=0)
    exercises_completed = Column(JSON)  # ["exercise-1", "exercise-2"]
    quiz_scores = Column(JSON)  # {quiz_id: score}


class Bookmark(Base):
    __tablename__ = "bookmarks"

    id = Column(String, primary_key=True, default=generate_uuid)
    user_id = Column(String, index=True, nullable=False)  # Foreign key to User.id
    chapter_id = Column(String, nullable=False)
    section_id = Column(String, nullable=True)
    notes = Column(Text, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())


class Highlight(Base):
    __tablename__ = "highlights"

    id = Column(String, primary_key=True, default=generate_uuid)
    user_id = Column(String, index=True, nullable=False)  # Foreign key to User.id
    chapter_id = Column(String, index=True, nullable=False)
    text_selection = Column(Text, nullable=False)
    start_offset = Column(Integer, nullable=False)  # Character offset in chapter
    end_offset = Column(Integer, nullable=False)
    annotation = Column(Text, nullable=True)
    created_at = Column(DateTime(timezone=True), server_default=func.now())