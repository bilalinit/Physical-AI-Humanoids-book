"""
Repository classes for ChatThread and ChatMessage operations
"""
from typing import List, Optional
from uuid import UUID
from datetime import datetime

from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select, update, delete, and_, func
from sqlalchemy.orm import selectinload

from ..models.chat_thread import ChatThread
from ..models.chat_message import ChatMessage
from ..models.user import User


class ChatThreadRepository:
    """
    Repository for ChatThread operations
    """

    def __init__(self, db: AsyncSession):
        self.db = db

    async def create_thread(self, user_id: str, title: str) -> ChatThread:
        """
        Create a new chat thread
        """
        thread = ChatThread(
            user_id=user_id,
            title=title
        )
        self.db.add(thread)
        await self.db.commit()
        await self.db.refresh(thread)
        return thread

    async def get_thread_by_id(self, thread_id: UUID, user_id: str) -> Optional[ChatThread]:
        """
        Get a chat thread by ID for a specific user
        """
        stmt = select(ChatThread).where(
            and_(
                ChatThread.id == thread_id,
                ChatThread.user_id == user_id
            )
        ).options(
            selectinload(ChatThread.messages)
        )
        result = await self.db.execute(stmt)
        return result.scalar_one_or_none()

    async def get_threads_by_user(self, user_id: str, limit: int = 50, offset: int = 0) -> tuple[List[ChatThread], int]:
        """
        Get all chat threads for a user with pagination
        """
        # Get total count
        count_stmt = select(func.count(ChatThread.id)).where(ChatThread.user_id == user_id)
        total_count_result = await self.db.execute(count_stmt)
        total_count = total_count_result.scalar()

        # Get threads with pagination
        stmt = select(ChatThread).where(ChatThread.user_id == user_id).order_by(
            ChatThread.updated_at.desc()
        ).offset(offset).limit(limit)

        result = await self.db.execute(stmt)
        threads = result.scalars().all()

        return threads, total_count

    async def update_thread_title(self, thread_id: UUID, user_id: str, title: str) -> Optional[ChatThread]:
        """
        Update the title of a chat thread
        """
        stmt = update(ChatThread).where(
            and_(
                ChatThread.id == thread_id,
                ChatThread.user_id == user_id
            )
        ).values(
            title=title,
            updated_at=func.now()
        ).returning(ChatThread)

        result = await self.db.execute(stmt)
        updated_thread = result.scalar_one_or_none()

        if updated_thread:
            await self.db.commit()
            await self.db.refresh(updated_thread)

        return updated_thread

    async def update_thread_timestamp(self, thread_id: UUID, user_id: str) -> bool:
        """
        Update the updated_at timestamp of a chat thread
        """
        stmt = update(ChatThread).where(
            and_(
                ChatThread.id == thread_id,
                ChatThread.user_id == user_id
            )
        ).values(
            updated_at=func.now()
        )

        result = await self.db.execute(stmt)
        updated = result.rowcount > 0

        if updated:
            await self.db.commit()

        return updated

    async def delete_thread(self, thread_id: UUID, user_id: str) -> bool:
        """
        Delete a chat thread
        """
        stmt = delete(ChatThread).where(
            and_(
                ChatThread.id == thread_id,
                ChatThread.user_id == user_id
            )
        )

        result = await self.db.execute(stmt)
        deleted = result.rowcount > 0

        if deleted:
            await self.db.commit()

        return deleted


class ChatMessageRepository:
    """
    Repository for ChatMessage operations
    """

    def __init__(self, db: AsyncSession):
        self.db = db

    async def create_message(self, thread_id: UUID, role: str, content: str, sources: Optional[dict] = None,
                           selected_text_used: bool = False) -> ChatMessage:
        """
        Create a new chat message
        """
        message = ChatMessage(
            thread_id=thread_id,
            role=role,
            content=content,
            sources=sources,
            selected_text_used=selected_text_used
        )
        self.db.add(message)
        await self.db.commit()
        await self.db.refresh(message)
        return message

    async def get_message_by_id(self, message_id: UUID, user_id: str) -> Optional[ChatMessage]:
        """
        Get a chat message by ID for a specific user
        """
        stmt = select(ChatMessage).join(ChatThread).where(
            and_(
                ChatMessage.id == message_id,
                ChatThread.user_id == user_id
            )
        )
        result = await self.db.execute(stmt)
        return result.scalar_one_or_none()

    async def get_messages_by_thread(self, thread_id: UUID, user_id: str, limit: int = 100, offset: int = 0) -> tuple[List[ChatMessage], int]:
        """
        Get all messages for a specific thread with pagination
        """
        # Get total count
        count_stmt = select(func.count(ChatMessage.id)).join(ChatThread).where(
            and_(
                ChatMessage.thread_id == thread_id,
                ChatThread.user_id == user_id
            )
        )
        total_count_result = await self.db.execute(count_stmt)
        total_count = total_count_result.scalar()

        # Get messages with pagination
        stmt = select(ChatMessage).where(
            ChatMessage.thread_id == thread_id
        ).order_by(
            ChatMessage.created_at.asc()
        ).offset(offset).limit(limit)

        result = await self.db.execute(stmt)
        messages = result.scalars().all()

        return messages, total_count

    async def get_messages_by_user(self, user_id: str, limit: int = 50, offset: int = 0) -> tuple[List[ChatMessage], int]:
        """
        Get all messages for a user across all threads with pagination
        """
        # Get total count
        count_stmt = select(func.count(ChatMessage.id)).join(ChatThread).where(
            ChatThread.user_id == user_id
        )
        total_count_result = await self.db.execute(count_stmt)
        total_count = total_count_result.scalar()

        # Get messages with pagination
        stmt = select(ChatMessage).join(ChatThread).where(
            ChatThread.user_id == user_id
        ).order_by(
            ChatMessage.created_at.desc()
        ).offset(offset).limit(limit)

        result = await self.db.execute(stmt)
        messages = result.scalars().all()

        return messages, total_count

    async def delete_message(self, message_id: UUID, user_id: str) -> bool:
        """
        Delete a chat message
        """
        stmt = delete(ChatMessage).join(ChatThread).where(
            and_(
                ChatMessage.id == message_id,
                ChatThread.user_id == user_id
            )
        )

        result = await self.db.execute(stmt)
        deleted = result.rowcount > 0

        if deleted:
            await self.db.commit()

        return deleted

    async def delete_messages_by_thread(self, thread_id: UUID, user_id: str) -> int:
        """
        Delete all messages in a thread
        """
        stmt = delete(ChatMessage).join(ChatThread).where(
            and_(
                ChatMessage.thread_id == thread_id,
                ChatThread.user_id == user_id
            )
        )

        result = await self.db.execute(stmt)
        deleted_count = result.rowcount

        if deleted_count > 0:
            await self.db.commit()

        return deleted_count