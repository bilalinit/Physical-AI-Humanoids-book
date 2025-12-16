"""
ChatMessage model for chat history persistence with Neon PostgreSQL database
"""
from sqlalchemy import Column, String, Text, DateTime, Boolean, ForeignKey, Index
from sqlalchemy.dialects.postgresql import UUID, JSONB
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from datetime import datetime
import uuid

from .base import Base

class ChatMessage(Base):
    __tablename__ = "chat_messages"

    id = Column("id", UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    thread_id = Column("thread_id", UUID(as_uuid=True), ForeignKey("chat_threads.id", ondelete="CASCADE"), nullable=False)
    role = Column("role", String(20), nullable=False)  # 'user' or 'assistant'
    content = Column("content", Text, nullable=False)
    sources = Column("sources", JSONB)  # JSONB column for document sources
    selected_text_used = Column("selected_text_used", Boolean, default=False)
    created_at = Column("created_at", DateTime(timezone=True), default=func.now())

    # Relationship to thread
    thread = relationship("ChatThread", back_populates="messages")

    def __repr__(self):
        return f"<ChatMessage(id={self.id}, thread_id={self.thread_id}, role={self.role})>"

    def to_dict(self):
        """Convert chat message object to dictionary for serialization"""
        return {
            "id": str(self.id),
            "thread_id": str(self.thread_id),
            "role": self.role,
            "content": self.content,
            "sources": self.sources,
            "selected_text_used": self.selected_text_used,
            "created_at": self.created_at.isoformat() if self.created_at else None
        }

# Create indexes
Index('idx_chat_messages_thread_id', ChatMessage.thread_id)
Index('idx_chat_messages_created_at', ChatMessage.created_at)