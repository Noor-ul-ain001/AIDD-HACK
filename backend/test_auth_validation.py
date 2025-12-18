"""
Test script to verify authentication validation works correctly.
Run this with the backend server running.
"""
import requests
import json

BASE_URL = "http://localhost:8000"

def test_signup(email: str, password: str, expected_success: bool):
    """Test signup endpoint with given credentials."""
    url = f"{BASE_URL}/api/auth/signup"
    payload = {
        "email": email,
        "password": password
    }

    try:
        response = requests.post(url, json=payload)
        success = response.status_code == 200

        status = "✓ PASS" if success == expected_success else "✗ FAIL"
        print(f"{status} | Email: {email:30} | Password: {password:20} | Status: {response.status_code} | {response.json().get('detail', 'Success')}")

    except requests.exceptions.ConnectionError:
        print("ERROR: Backend server is not running. Start it with: cd backend && uvicorn src.main:app --reload")
        return False
    except Exception as e:
        print(f"ERROR: {e}")
        return False

    return True

if __name__ == "__main__":
    print("=" * 120)
    print("AUTHENTICATION VALIDATION TESTS")
    print("=" * 120)

    print("\n--- Invalid Password Tests (should fail) ---")
    test_signup("valid@gmail.com", "password123", False)  # No uppercase, common password
    test_signup("valid@gmail.com", "Test123", False)  # Too short, no special char
    test_signup("valid@gmail.com", "NoSpecial123", False)  # No special character
    test_signup("valid@gmail.com", "nouppercas3!", False)  # No uppercase
    test_signup("valid@gmail.com", "NOLOWERCASE3!", False)  # No lowercase
    test_signup("valid@gmail.com", "NoNumbers!", False)  # No numbers

    print("\n--- Invalid Email Tests (should fail) ---")
    test_signup("test@gmail.com", "ValidP@ss123", False)  # Blocked local part
    test_signup("admin@gmail.com", "ValidP@ss123", False)  # Blocked local part
    test_signup("dummy@gmail.com", "ValidP@ss123", False)  # Blocked local part
    test_signup("user@tempmail.com", "ValidP@ss123", False)  # Disposable email
    test_signup("user@10minutemail.com", "ValidP@ss123", False)  # Disposable email

    print("\n--- Valid Credentials Tests (should succeed) ---")
    test_signup("realuser@gmail.com", "SecureP@ss123", True)
    test_signup("john.doe@company.com", "MyP@ssw0rd!", True)
    test_signup("jane_smith@university.edu", "Strong#Pass99", True)

    print("\n" + "=" * 120)
    print("NOTE: If you see 'User already exists' errors, the tests previously succeeded.")
    print("=" * 120)
