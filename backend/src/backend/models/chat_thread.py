"""
ChatThread model for chat history persistence with Neon PostgreSQL database
"""
from sqlalchemy import Column, String, Text, DateTime, ForeignKey, Index
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from datetime import datetime
import uuid

from .base import Base

class ChatThread(Base):
    __tablename__ = "chat_threads"

    id = Column("id", UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column("user_id", String, ForeignKey("user.id", ondelete="CASCADE"), nullable=False)
    title = Column("title", String(200), nullable=False)
    created_at = Column("created_at", DateTime(timezone=True), default=func.now())
    updated_at = Column("updated_at", DateTime(timezone=True), default=func.now(), onupdate=func.now())

    # Relationship to user
    user = relationship("User", back_populates="chat_threads")

    # Relationship to messages
    messages = relationship("ChatMessage", back_populates="thread", cascade="all, delete-orphan", order_by="ChatMessage.created_at")

    def __repr__(self):
        return f"<ChatThread(id={self.id}, user_id={self.user_id}, title={self.title})>"

    def to_dict(self):
        """Convert chat thread object to dictionary for serialization"""
        return {
            "id": str(self.id),
            "user_id": self.user_id,
            "title": self.title,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "updated_at": self.updated_at.isoformat() if self.updated_at else None
        }

# Create indexes
Index('idx_chat_threads_user_id', ChatThread.user_id)
Index('idx_chat_threads_updated_at', ChatThread.updated_at)