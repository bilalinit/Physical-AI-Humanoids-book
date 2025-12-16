"""
Chat thread service for persistent chat history with Neon PostgreSQL database
"""
from typing import List, Optional, Dict, Any
from uuid import UUID
import logging

from sqlalchemy.ext.asyncio import AsyncSession

from .repositories import ChatThreadRepository, ChatMessageRepository
from ..models.chat_thread import ChatThread
from ..models.chat_message import ChatMessage


logger = logging.getLogger(__name__)


class ChatThreadService:
    """
    Service to handle chat thread and message operations
    """

    def __init__(self, db: AsyncSession):
        self.db = db
        self.thread_repo = ChatThreadRepository(db)
        self.message_repo = ChatMessageRepository(db)

    async def create_chat_thread(self, user_id: str, title: Optional[str] = None) -> ChatThread:
        """
        Create a new chat thread for a user

        Args:
            user_id: The ID of the user creating the thread
            title: Optional title for the thread; will be auto-generated if not provided

        Returns:
            The created ChatThread object
        """
        try:
            # Generate a default title if not provided
            if not title:
                title = "New Conversation"

            # Create the chat thread
            thread = await self.thread_repo.create_thread(user_id, title)

            logger.info(f"Created chat thread {thread.id} for user {user_id}")

            return thread
        except Exception as e:
            logger.error(f"Error creating chat thread for user {user_id}: {str(e)}")
            raise

    async def get_thread_by_id(self, thread_id: UUID, user_id: str) -> Optional[ChatThread]:
        """
        Get a specific chat thread for a user

        Args:
            thread_id: The ID of the thread to retrieve
            user_id: The ID of the user requesting the thread

        Returns:
            The ChatThread object if found and belongs to the user, None otherwise
        """
        try:
            thread = await self.thread_repo.get_thread_by_id(thread_id, user_id)

            if thread:
                logger.info(f"Retrieved chat thread {thread_id} for user {user_id}")
            else:
                logger.warning(f"Chat thread {thread_id} not found for user {user_id}")

            return thread
        except Exception as e:
            logger.error(f"Error retrieving chat thread {thread_id} for user {user_id}: {str(e)}")
            raise

    async def get_user_threads(self, user_id: str, limit: int = 50, offset: int = 0) -> tuple[List[ChatThread], int]:
        """
        Get all chat threads for a user

        Args:
            user_id: The ID of the user whose threads to retrieve
            limit: Maximum number of threads to return
            offset: Number of threads to skip

        Returns:
            Tuple of (list of threads, total count)
        """
        try:
            threads, total_count = await self.thread_repo.get_threads_by_user(user_id, limit, offset)

            logger.info(f"Retrieved {len(threads)} chat threads for user {user_id} (total: {total_count})")

            return threads, total_count
        except Exception as e:
            logger.error(f"Error retrieving chat threads for user {user_id}: {str(e)}")
            raise

    async def update_thread_title(self, thread_id: UUID, user_id: str, title: str) -> Optional[ChatThread]:
        """
        Update the title of a chat thread

        Args:
            thread_id: The ID of the thread to update
            user_id: The ID of the user who owns the thread
            title: The new title

        Returns:
            Updated ChatThread object if successful, None if thread not found
        """
        try:
            if not title or len(title) > 200:
                raise ValueError("Title must be between 1 and 200 characters")

            updated_thread = await self.thread_repo.update_thread_title(thread_id, user_id, title)

            if updated_thread:
                logger.info(f"Updated title for chat thread {thread_id}")
            else:
                logger.warning(f"Chat thread {thread_id} not found for user {user_id}")

            return updated_thread
        except Exception as e:
            logger.error(f"Error updating chat thread {thread_id} title: {str(e)}")
            raise

    async def delete_thread(self, thread_id: UUID, user_id: str) -> bool:
        """
        Delete a chat thread and all its messages

        Args:
            thread_id: The ID of the thread to delete
            user_id: The ID of the user who owns the thread

        Returns:
            True if thread was deleted, False if not found
        """
        try:
            # Delete the thread (messages will be deleted automatically due to CASCADE)
            deleted = await self.thread_repo.delete_thread(thread_id, user_id)

            if deleted:
                logger.info(f"Deleted chat thread {thread_id} for user {user_id}")
            else:
                logger.warning(f"Chat thread {thread_id} not found for user {user_id}")

            return deleted
        except Exception as e:
            logger.error(f"Error deleting chat thread {thread_id} for user {user_id}: {str(e)}")
            raise

    async def add_message_to_thread(
        self,
        thread_id: UUID,
        user_id: str,
        role: str,
        content: str,
        sources: Optional[Dict] = None,
        selected_text_used: bool = False
    ) -> ChatMessage:
        """
        Add a message to a chat thread

        Args:
            thread_id: The ID of the thread to add the message to
            user_id: The ID of the user adding the message (for validation)
            role: The role of the message sender ('user' or 'assistant')
            content: The message content
            sources: Optional sources referenced in the message
            selected_text_used: Whether selected text was used in this message

        Returns:
            The created ChatMessage object
        """
        try:
            # Validate input
            if role not in ['user', 'assistant']:
                raise ValueError("Role must be either 'user' or 'assistant'")

            if not content:
                raise ValueError("Content cannot be empty")

            # Verify that the thread belongs to the user
            thread = await self.thread_repo.get_thread_by_id(thread_id, user_id)
            if not thread:
                raise ValueError(f"Thread {thread_id} does not exist or does not belong to user {user_id}")

            # Create the message
            message = await self.message_repo.create_message(
                thread_id=thread_id,
                role=role,
                content=content,
                sources=sources,
                selected_text_used=selected_text_used
            )

            # Update the thread's updated_at timestamp
            await self.thread_repo.update_thread_timestamp(thread_id, user_id)

            logger.info(f"Added message {message.id} to chat thread {thread_id}")

            return message
        except Exception as e:
            logger.error(f"Error adding message to chat thread {thread_id}: {str(e)}")
            raise

    async def get_thread_messages(
        self,
        thread_id: UUID,
        user_id: str,
        limit: int = 100,
        offset: int = 0
    ) -> tuple[List[ChatMessage], int]:
        """
        Get all messages in a specific thread

        Args:
            thread_id: The ID of the thread
            user_id: The ID of the user requesting the messages
            limit: Maximum number of messages to return
            offset: Number of messages to skip

        Returns:
            Tuple of (list of messages, total count)
        """
        try:
            # Verify that the thread belongs to the user
            thread = await self.thread_repo.get_thread_by_id(thread_id, user_id)
            if not thread:
                raise ValueError(f"Thread {thread_id} does not exist or does not belong to user {user_id}")

            messages, total_count = await self.message_repo.get_messages_by_thread(thread_id, user_id, limit, offset)

            logger.info(f"Retrieved {len(messages)} messages from chat thread {thread_id}")

            return messages, total_count
        except Exception as e:
            logger.error(f"Error retrieving messages from chat thread {thread_id}: {str(e)}")
            raise

    async def generate_thread_title_from_first_message(self, thread_id: UUID, user_id: str, first_message_content: str) -> Optional[ChatThread]:
        """
        Generate and set a thread title based on the first message content

        Args:
            thread_id: The ID of the thread to update
            user_id: The ID of the user who owns the thread
            first_message_content: The content of the first message to base the title on

        Returns:
            Updated ChatThread object if successful, None if thread not found
        """
        try:
            # Generate a title based on the first few words of the message
            # Take the first 50 characters and append "..." if it's longer
            if len(first_message_content) > 50:
                title = first_message_content[:47] + "..."
            else:
                title = first_message_content.strip()

            # Ensure the title is not empty
            if not title.strip():
                title = "New Conversation"

            # Update the thread title
            updated_thread = await self.thread_repo.update_thread_title(thread_id, user_id, title)

            if updated_thread:
                logger.info(f"Auto-generated title for thread {thread_id}: {title}")
            else:
                logger.warning(f"Could not update title for thread {thread_id}")

            return updated_thread
        except Exception as e:
            logger.error(f"Error generating thread title for {thread_id}: {str(e)}")
            raise


# Global service instance function
def get_chat_thread_service(db: AsyncSession) -> ChatThreadService:
    """
    Get a chat thread service instance with the given database session
    """
    return ChatThreadService(db)