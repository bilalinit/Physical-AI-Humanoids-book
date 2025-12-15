"""
ChatHistory model for Better Auth + Neon integration
"""
from sqlalchemy import Column, Integer, String, Text, DateTime, ForeignKey
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import relationship
from datetime import datetime

# Base class for SQLAlchemy models
Base = declarative_base()

class ChatHistory(Base):
    __tablename__ = "chat_history"

    id = Column("id", Integer, primary_key=True, autoincrement=True)  # Auto-incrementing integer
    user_id = Column("userId", String, ForeignKey("user.id", ondelete="CASCADE"), nullable=False)  # References User.id with ON DELETE CASCADE
    message = Column("message", Text, nullable=False)  # User's question/message to the chatbot
    response = Column("response", Text, nullable=False)  # AI assistant's response
    selected_text = Column("selectedText", Text)  # Text selected from documentation (optional context)
    created_at = Column("createdAt", DateTime, nullable=False, default=datetime.utcnow)  # Message timestamp (default: NOW())

    # Relationship
    user = relationship("User", back_populates="chat_histories")

    def __repr__(self):
        return f"<ChatHistory(id={self.id}, userId={self.user_id}, createdAt={self.created_at})>"

    def to_dict(self):
        """Convert chat history object to dictionary for serialization"""
        return {
            "id": self.id,
            "userId": self.user_id,
            "message": self.message,
            "response": self.response,
            "selectedText": self.selected_text,
            "createdAt": self.created_at.isoformat() if self.created_at else None
        }