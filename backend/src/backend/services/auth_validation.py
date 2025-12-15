"""
Session validation service for Better Auth integration
"""
import os
import httpx
from typing import Optional, Dict, Any
from datetime import datetime
import logging

logger = logging.getLogger(__name__)

class AuthValidationService:
    """
    Service to validate user sessions by calling the auth server
    """

    def __init__(self):
        self.auth_server_url = os.getenv("AUTH_SERVER_URL", "http://localhost:3001")
        self.session_validation_endpoint = f"{self.auth_server_url}/api/auth/get-session"

    async def validate_session(self, session_token: str) -> Optional[Dict[str, Any]]:
        """
        Validate a session token by calling the auth server

        Args:
            session_token: The session token to validate

        Returns:
            User data if session is valid, None if invalid
        """
        try:
            # Make request to auth server to validate session
            async with httpx.AsyncClient() as client:
                response = await client.get(
                    self.session_validation_endpoint,
                    headers={
                        "Authorization": f"Bearer {session_token}",
                        "Content-Type": "application/json"
                    },
                    timeout=10.0  # 10 second timeout
                )

                if response.status_code == 200:
                    data = response.json()
                    session_data = data.get("session", {})
                    user_data = session_data.get("user", {})

                    # Return user data with session info
                    return {
                        "user_id": user_data.get("id"),
                        "email": user_data.get("email"),
                        "name": user_data.get("name"),
                        "education_level": user_data.get("educationLevel"),
                        "programming_experience": user_data.get("programmingExperience"),
                        "software_background": user_data.get("softwareBackground"),
                        "hardware_background": user_data.get("hardwareBackground"),
                        "robotics_background": user_data.get("roboticsBackground"),
                        "created_at": user_data.get("createdAt"),
                        "expires_at": session_data.get("expiresAt")
                    }
                else:
                    logger.warning(f"Session validation failed with status {response.status_code}")
                    return None

        except httpx.RequestError as e:
            logger.error(f"Request error during session validation: {str(e)}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during session validation: {str(e)}")
            return None

    async def validate_session_from_request(self, request) -> Optional[Dict[str, Any]]:
        """
        Validate session from a FastAPI request object

        Args:
            request: FastAPI request object

        Returns:
            User data if session is valid, None if invalid
        """
        # Try to get session token from Authorization header
        auth_header = request.headers.get("Authorization")

        if auth_header and auth_header.startswith("Bearer "):
            session_token = auth_header[7:]  # Remove "Bearer " prefix
            return await self.validate_session(session_token)

        # Try to get session token from cookies (for Better Auth cookie-based auth)
        session_token = request.cookies.get("better-auth.session_token")
        if session_token:
            return await self.validate_session(session_token)

        return None

# Global instance
auth_validator = AuthValidationService()