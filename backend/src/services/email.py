"""
Email service for sending verification and reset emails.
Uses SendGrid for production or prints to console for development.
"""
import os
import smtplib
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from typing import Optional
from ..core.config import settings

class EmailService:
    def __init__(self):
        self.from_email = os.getenv("FROM_EMAIL", "noreply@physicalai.edu")
        self.smtp_host = os.getenv("SMTP_HOST", "smtp.gmail.com")
        self.smtp_port = int(os.getenv("SMTP_PORT", "587"))
        self.smtp_user = os.getenv("SMTP_USER")
        self.smtp_password = os.getenv("SMTP_PASSWORD")
        self.base_url = settings.BETTER_AUTH_URL

    async def send_email(self, to_email: str, subject: str, html_content: str) -> bool:
        """Send an email. Falls back to console logging if SMTP not configured."""

        # If SMTP not configured, just log to console (development mode)
        if not self.smtp_user or not self.smtp_password:
            print("=" * 60)
            print(f"📧 EMAIL (Development Mode)")
            print(f"To: {to_email}")
            print(f"Subject: {subject}")
            print(f"\n{html_content}\n")
            print("=" * 60)
            return True

        try:
            # Create message
            message = MIMEMultipart('alternative')
            message['Subject'] = subject
            message['From'] = self.from_email
            message['To'] = to_email

            # Attach HTML content
            html_part = MIMEText(html_content, 'html')
            message.attach(html_part)

            # Send email
            with smtplib.SMTP(self.smtp_host, self.smtp_port) as server:
                server.starttls()
                server.login(self.smtp_user, self.smtp_password)
                server.send_message(message)

            return True
        except Exception as e:
            print(f"Error sending email: {e}")
            return False

    async def send_verification_email(self, email: str, token: str) -> bool:
        """Send email verification link."""
        verification_link = f"{self.base_url}/verify-email?token={token}"

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <style>
                body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
                .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
                .button {{
                    display: inline-block;
                    padding: 12px 24px;
                    background-color: #007bff;
                    color: white;
                    text-decoration: none;
                    border-radius: 4px;
                    margin: 20px 0;
                }}
                .footer {{ margin-top: 30px; font-size: 12px; color: #666; }}
            </style>
        </head>
        <body>
            <div class="container">
                <h2>Verify Your Email Address</h2>
                <p>Thank you for signing up for Physical AI & Humanoid Robotics platform!</p>
                <p>Please click the button below to verify your email address:</p>
                <a href="{verification_link}" class="button">Verify Email</a>
                <p>Or copy and paste this link into your browser:</p>
                <p style="word-break: break-all; color: #007bff;">{verification_link}</p>
                <p class="footer">
                    This link will expire in 24 hours. If you didn't create an account, you can safely ignore this email.
                </p>
            </div>
        </body>
        </html>
        """

        return await self.send_email(
            email,
            "Verify your email address",
            html_content
        )

    async def send_password_reset_email(self, email: str, token: str) -> bool:
        """Send password reset link."""
        reset_link = f"{self.base_url}/reset-password?token={token}"

        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <style>
                body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
                .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
                .button {{
                    display: inline-block;
                    padding: 12px 24px;
                    background-color: #dc3545;
                    color: white;
                    text-decoration: none;
                    border-radius: 4px;
                    margin: 20px 0;
                }}
                .footer {{ margin-top: 30px; font-size: 12px; color: #666; }}
            </style>
        </head>
        <body>
            <div class="container">
                <h2>Reset Your Password</h2>
                <p>We received a request to reset your password for your Physical AI account.</p>
                <p>Click the button below to reset your password:</p>
                <a href="{reset_link}" class="button">Reset Password</a>
                <p>Or copy and paste this link into your browser:</p>
                <p style="word-break: break-all; color: #dc3545;">{reset_link}</p>
                <p class="footer">
                    This link will expire in 1 hour. If you didn't request a password reset, you can safely ignore this email.
                </p>
            </div>
        </body>
        </html>
        """

        return await self.send_email(
            email,
            "Reset your password",
            html_content
        )

    async def send_2fa_code(self, email: str, code: str) -> bool:
        """Send 2FA verification code."""
        html_content = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <style>
                body {{ font-family: Arial, sans-serif; line-height: 1.6; color: #333; }}
                .container {{ max-width: 600px; margin: 0 auto; padding: 20px; }}
                .code {{
                    font-size: 32px;
                    font-weight: bold;
                    color: #007bff;
                    letter-spacing: 8px;
                    text-align: center;
                    margin: 30px 0;
                }}
                .footer {{ margin-top: 30px; font-size: 12px; color: #666; }}
            </style>
        </head>
        <body>
            <div class="container">
                <h2>Your Verification Code</h2>
                <p>Enter this code to complete your login:</p>
                <div class="code">{code}</div>
                <p class="footer">
                    This code will expire in 10 minutes. If you didn't request this code, please secure your account immediately.
                </p>
            </div>
        </body>
        </html>
        """

        return await self.send_email(
            email,
            "Your verification code",
            html_content
        )

# Singleton instance
email_service = EmailService()
