"""
Chat history service for Better Auth + Neon integration
"""
from sqlalchemy.orm import Session
from typing import List, Optional
from datetime import datetime
import logging

from ..models.chat_history import ChatHistory
from ..models.user import User

logger = logging.getLogger(__name__)

class ChatHistoryService:
    """
    Service to handle chat history operations
    """

    def __init__(self, db: Session):
        self.db = db

    def save_message(self, user_id: str, message: str, response: str, selected_text: Optional[str] = None) -> ChatHistory:
        """
        Save a chat message to the database

        Args:
            user_id: The ID of the user who sent the message
            message: The user's message/question
            response: The AI's response
            selected_text: Optional selected text from documentation

        Returns:
            The saved ChatHistory object
        """
        try:
            # Validate input
            if not message or not response:
                raise ValueError("Message and response are required")

            if len(message) > 5000 or len(response) > 10000:
                raise ValueError("Message or response exceeds maximum length")

            if selected_text and len(selected_text) > 2000:
                raise ValueError("Selected text exceeds maximum length")

            # Create chat history record
            chat_history = ChatHistory(
                user_id=user_id,
                message=message,
                response=response,
                selected_text=selected_text
            )

            self.db.add(chat_history)
            self.db.commit()
            self.db.refresh(chat_history)

            logger.info(f"Saved chat message for user {user_id}: {chat_history.id}")

            return chat_history
        except Exception as e:
            logger.error(f"Error saving chat message: {str(e)}")
            self.db.rollback()
            raise

    def get_user_history(self, user_id: str, limit: int = 50, offset: int = 0) -> List[ChatHistory]:
        """
        Retrieve chat history for a specific user

        Args:
            user_id: The ID of the user whose history to retrieve
            limit: Maximum number of messages to return
            offset: Number of messages to skip

        Returns:
            List of ChatHistory objects
        """
        try:
            # Query chat history for the user with pagination
            chat_messages = (
                self.db.query(ChatHistory)
                .filter(ChatHistory.user_id == user_id)
                .order_by(ChatHistory.created_at.desc())
                .offset(offset)
                .limit(limit)
                .all()
            )

            logger.info(f"Retrieved {len(chat_messages)} chat messages for user {user_id}")

            return chat_messages
        except Exception as e:
            logger.error(f"Error retrieving chat history for user {user_id}: {str(e)}")
            raise

    def get_message_count(self, user_id: str) -> int:
        """
        Get the total count of chat messages for a user

        Args:
            user_id: The ID of the user

        Returns:
            Total count of messages
        """
        try:
            count = (
                self.db.query(ChatHistory)
                .filter(ChatHistory.user_id == user_id)
                .count()
            )

            return count
        except Exception as e:
            logger.error(f"Error counting chat messages for user {user_id}: {str(e)}")
            raise

    def delete_message(self, message_id: int, user_id: str) -> bool:
        """
        Delete a specific chat message

        Args:
            message_id: The ID of the message to delete
            user_id: The ID of the user who owns the message

        Returns:
            True if message was deleted, False if not found
        """
        try:
            # Find the message for the specific user
            chat_message = (
                self.db.query(ChatHistory)
                .filter(ChatHistory.id == message_id, ChatHistory.user_id == user_id)
                .first()
            )

            if not chat_message:
                logger.warning(f"Message {message_id} not found for user {user_id}")
                return False

            # Delete the message
            self.db.delete(chat_message)
            self.db.commit()

            logger.info(f"Deleted chat message {message_id} for user {user_id}")

            return True
        except Exception as e:
            logger.error(f"Error deleting chat message {message_id} for user {user_id}: {str(e)}")
            self.db.rollback()
            raise

    def delete_user_history(self, user_id: str) -> int:
        """
        Delete all chat history for a specific user

        Args:
            user_id: The ID of the user whose history to delete

        Returns:
            Number of messages deleted
        """
        try:
            # Find all messages for the user
            messages = self.db.query(ChatHistory).filter(ChatHistory.user_id == user_id).all()

            # Delete each message
            deleted_count = 0
            for message in messages:
                self.db.delete(message)
                deleted_count += 1

            self.db.commit()

            logger.info(f"Deleted {deleted_count} chat messages for user {user_id}")

            return deleted_count
        except Exception as e:
            logger.error(f"Error deleting chat history for user {user_id}: {str(e)}")
            self.db.rollback()
            raise

    def get_recent_messages(self, user_id: str, hours: int = 24) -> List[ChatHistory]:
        """
        Get recent chat messages for a user within the specified time period

        Args:
            user_id: The ID of the user
            hours: Number of hours to look back

        Returns:
            List of recent ChatHistory objects
        """
        from datetime import datetime, timedelta

        try:
            cutoff_time = datetime.utcnow() - timedelta(hours=hours)

            recent_messages = (
                self.db.query(ChatHistory)
                .filter(
                    ChatHistory.user_id == user_id,
                    ChatHistory.created_at >= cutoff_time
                )
                .order_by(ChatHistory.created_at.desc())
                .all()
            )

            return recent_messages
        except Exception as e:
            logger.error(f"Error retrieving recent chat messages for user {user_id}: {str(e)}")
            raise

# Global service instance function
def get_chat_history_service(db: Session) -> ChatHistoryService:
    """
    Get a chat history service instance with the given database session
    """
    return ChatHistoryService(db)