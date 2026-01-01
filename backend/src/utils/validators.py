"""
Validation utilities for authentication and user input.
"""
import re
from typing import Tuple
from pydantic import EmailStr, validator

# Common weak passwords to reject
WEAK_PASSWORDS = {
    "password", "password123", "12345678", "qwerty", "abc123",
    "password1", "123456", "test123", "admin123", "welcome",
    "letmein", "monkey", "dragon", "master", "sunshine",
    "princess", "starwars", "654321", "batman", "superman",
    "iloveyou", "trustno1", "login", "admin", "root"
}

# Disposable email domains to block
DISPOSABLE_EMAIL_DOMAINS = {
    "tempmail.com", "throwaway.email", "guerrillamail.com",
    "mailinator.com", "10minutemail.com", "trashmail.com",
    "maildrop.cc", "temp-mail.org", "yopmail.com", "fakeinbox.com",
    "getnada.com", "throwawaymail.com", "temp-mail.io"
}


def validate_password_strength(password: str) -> Tuple[bool, str]:
    """
    Validate password strength with strict requirements.

    Requirements:
    - Minimum 8 characters
    - At least one uppercase letter
    - At least one lowercase letter
    - At least one number
    - At least one special character
    - Not a common weak password

    Returns:
        Tuple[bool, str]: (is_valid, error_message)
    """

    # Check minimum length
    if len(password) < 8:
        return False, "Password must be at least 8 characters long"

    # Check maximum length (prevent DoS)
    if len(password) > 128:
        return False, "Password must not exceed 128 characters"

    # Check for uppercase letter
    if not re.search(r'[A-Z]', password):
        return False, "Password must contain at least one uppercase letter"

    # Check for lowercase letter
    if not re.search(r'[a-z]', password):
        return False, "Password must contain at least one lowercase letter"

    # Check for digit
    if not re.search(r'\d', password):
        return False, "Password must contain at least one number"

    # Check for special character
    if not re.search(r'[!@#$%^&*()_+\-=\[\]{};:\'",.<>?/\\|`~]', password):
        return False, "Password must contain at least one special character (!@#$%^&* etc.)"

    # Check against weak password list
    password_lower = password.lower()
    if password_lower in WEAK_PASSWORDS:
        return False, "This password is too common. Please choose a stronger password"

    # Check for common patterns - same character repeated 4+ times (too strict at 3)
    if re.search(r'(.)\1{3,}', password):
        return False, "Password should not contain the same character repeated multiple times"

    return True, ""


def validate_email(email: str) -> Tuple[bool, str]:
    """
    Validate email address beyond basic format checking.

    Returns:
        Tuple[bool, str]: (is_valid, error_message)
    """

    # Basic format validation (this is done by Pydantic EmailStr already)
    email_pattern = r'^[a-zA-Z0-9._%+-]+@[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$'
    if not re.match(email_pattern, email):
        return False, "Invalid email format"

    # Extract domain
    domain = email.split('@')[-1].lower()
    local_part = email.split('@')[0].lower()

    # Check against disposable email domains
    if domain in DISPOSABLE_EMAIL_DOMAINS:
        return False, "Temporary or disposable email addresses are not allowed"

    # Check for valid domain format
    if not re.match(r'^[a-zA-Z0-9.-]+\.[a-zA-Z]{2,}$', domain):
        return False, "Invalid email domain"

    # Check for known dummy/local testing domains
    dummy_domains = {'localhost', 'localdomain', 'test', 'example', 'invalid', 'localhost.localdomain'}
    if domain in dummy_domains or domain.endswith('.local') or domain.endswith('.test') or domain.endswith('.invalid'):
        return False, "Please use a valid email address (not test domains)"

    # More comprehensive check for dummy-like emails using patterns
    dummy_patterns = [
        r'^test',           # starts with test
        r'^dummy',          # starts with dummy
        r'^fake',           # starts with fake
        r'^example',        # starts with example
        r'^sample',         # starts with sample
        r'^temp',           # starts with temp
        r'^user\d+',        # user with numbers like user123
        r'^admin',          # starts with admin
        r'^root',           # starts with root
        r'^email',          # starts with email
        r'^mail',           # starts with mail
        r'^myemail',        # starts with myemail
        r'^a+$',            # only a's (like aaaaaa@domain.com)
        r'^x+$',            # only x's (like xxxxxx@domain.com)
        r'^[0-9]+$',        # only numbers
    ]

    for pattern in dummy_patterns:
        if re.match(pattern, local_part):
            return False, "Please use a valid personal email address"

    # Prevent obviously fake emails (exact matches)
    blocked_local_parts = ['test', 'admin', 'noreply', 'no-reply', 'dummy', 'fake', 'sample', 'example',
                          'user', 'username', 'your-email', 'your_email', 'email', 'mail', 'contact', 'info']
    if local_part in blocked_local_parts:
        return False, "Please use a valid email address (not test/dummy accounts)"

    return True, ""


def validate_password_match(password: str, confirm_password: str) -> Tuple[bool, str]:
    """
    Validate that password and confirmation match.

    Returns:
        Tuple[bool, str]: (is_valid, error_message)
    """
    if password != confirm_password:
        return False, "Passwords do not match"

    return True, ""
