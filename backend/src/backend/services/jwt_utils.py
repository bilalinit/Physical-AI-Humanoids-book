"""
JWT validation utilities for user authentication
"""
import os
from datetime import datetime
from typing import Optional, Dict, Any
import logging

from jose import jwt, JWTError
from dotenv import load_dotenv

load_dotenv()

logger = logging.getLogger(__name__)

class JWTValidationService:
    """
    Service to validate JWT tokens for user authentication
    """

    def __init__(self):
        self.secret_key = os.getenv("AUTH_JWT_SECRET")
        self.algorithm = os.getenv("AUTH_JWT_ALGORITHM", "HS256")

        if not self.secret_key:
            logger.warning("AUTH_JWT_SECRET environment variable not set. JWT validation will not work properly.")

    async def validate_jwt_token(self, token: str) -> Optional[Dict[str, Any]]:
        """
        Validate a JWT token and return user data if valid

        Args:
            token: JWT token to validate

        Returns:
            User data if token is valid, None if invalid
        """
        if not self.secret_key:
            logger.error("JWT secret key not configured")
            return None

        try:
            # Decode the JWT token
            payload = jwt.decode(token, self.secret_key, algorithms=[self.algorithm])

            # Extract user information from the payload
            user_data = {
                "user_id": payload.get("sub"),  # Subject (typically user ID)
                "email": payload.get("email"),
                "name": payload.get("name"),
                "exp": payload.get("exp"),  # Expiration time
                "iat": payload.get("iat"),  # Issued at time
                "session_id": payload.get("session_id")
            }

            # Check if token is expired
            exp = payload.get("exp")
            if exp and datetime.fromtimestamp(exp) < datetime.utcnow():
                logger.warning("JWT token is expired")
                return None

            return user_data

        except JWTError as e:
            logger.error(f"JWT validation error: {str(e)}")
            return None
        except Exception as e:
            logger.error(f"Unexpected error during JWT validation: {str(e)}")
            return None

    async def validate_jwt_from_request(self, request) -> Optional[Dict[str, Any]]:
        """
        Validate JWT token from a FastAPI request object

        Args:
            request: FastAPI request object

        Returns:
            User data if token is valid, None if invalid
        """
        # Try to get JWT token from Authorization header
        auth_header = request.headers.get("Authorization")

        if auth_header and auth_header.startswith("Bearer "):
            token = auth_header[7:]  # Remove "Bearer " prefix
            return await self.validate_jwt_token(token)

        return None

    def create_jwt_token(self, user_data: Dict[str, Any]) -> Optional[str]:
        """
        Create a JWT token for a user

        Args:
            user_data: User data to include in the token

        Returns:
            JWT token string if successful, None if failed
        """
        if not self.secret_key:
            logger.error("JWT secret key not configured")
            return None

        try:
            # Prepare payload with user data
            payload = {
                "sub": user_data.get("user_id"),  # Subject (user ID)
                "email": user_data.get("email"),
                "name": user_data.get("name"),
                "session_id": user_data.get("session_id"),
                "iat": datetime.utcnow(),  # Issued at time
                "exp": datetime.utcnow().timestamp() + 86400  # Expire in 24 hours
            }

            # Create JWT token
            token = jwt.encode(payload, self.secret_key, algorithm=self.algorithm)
            return token

        except Exception as e:
            logger.error(f"Error creating JWT token: {str(e)}")
            return None

# Global instance
jwt_validator = JWTValidationService()