"""
Chat history API endpoints for Better Auth + Neon integration
"""
from fastapi import APIRouter, Depends, HTTPException, Request
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import logging

from sqlalchemy.orm import Session
from ..database import get_db, engine
from ..models.chat_history import ChatHistory
from ..models.user import User  # Import the User model
from ..services.auth_validation import auth_validator

logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/api/chat-history", tags=["chat-history"])

# Pydantic models for request/response
class ChatHistoryCreate(BaseModel):
    message: str
    response: str
    selected_text: Optional[str] = None

class ChatHistoryResponse(BaseModel):
    id: int
    user_id: str
    message: str
    response: str
    selected_text: Optional[str]
    created_at: datetime

    class Config:
        from_attributes = True

class ChatHistoryListResponse(BaseModel):
    messages: List[ChatHistoryResponse]
    total: int

@router.post("/", response_model=ChatHistoryResponse)
async def save_chat_message(
    request: Request,
    chat_data: ChatHistoryCreate,
    db: Session = Depends(get_db)
):
    """
    Save a chat message to the database for authenticated users
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    # Validate input
    if not chat_data.message or not chat_data.response:
        raise HTTPException(status_code=400, detail="Message and response are required")

    if len(chat_data.message) > 5000 or len(chat_data.response) > 10000:
        raise HTTPException(status_code=400, detail="Message or response exceeds maximum length")

    if chat_data.selected_text and len(chat_data.selected_text) > 2000:
        raise HTTPException(status_code=400, detail="Selected text exceeds maximum length")

    try:
        # Create chat history record
        chat_history = ChatHistory(
            user_id=user_data["user_id"],
            message=chat_data.message,
            response=chat_data.response,
            selected_text=chat_data.selected_text
        )

        db.add(chat_history)
        db.commit()
        db.refresh(chat_history)

        logger.info(f"Saved chat message for user {user_data['user_id']}: {chat_history.id}")

        return ChatHistoryResponse(
            id=chat_history.id,
            user_id=chat_history.user_id,
            message=chat_history.message,
            response=chat_history.response,
            selected_text=chat_history.selected_text,
            created_at=chat_history.created_at
        )
    except Exception as e:
        logger.error(f"Error saving chat message: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/", response_model=ChatHistoryListResponse)
async def get_chat_history(
    request: Request,
    limit: int = 50,
    offset: int = 0,
    db: Session = Depends(get_db)
):
    """
    Retrieve chat history for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    try:
        # Query chat history for the user
        query = db.query(ChatHistory).filter(ChatHistory.user_id == user_data["user_id"])

        # Get total count
        total = query.count()

        # Apply pagination
        chat_messages = query.order_by(ChatHistory.created_at.desc()).offset(offset).limit(limit).all()

        # Convert to response format
        messages = [
            ChatHistoryResponse(
                id=msg.id,
                user_id=msg.user_id,
                message=msg.message,
                response=msg.response,
                selected_text=msg.selected_text,
                created_at=msg.created_at
            )
            for msg in chat_messages
        ]

        logger.info(f"Retrieved {len(messages)} chat messages for user {user_data['user_id']}")

        return ChatHistoryListResponse(
            messages=messages,
            total=total
        )
    except Exception as e:
        logger.error(f"Error retrieving chat history: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.delete("/{message_id}")
async def delete_chat_message(
    request: Request,
    message_id: int,
    db: Session = Depends(get_db)
):
    """
    Delete a specific chat message for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    try:
        # Find the message
        chat_message = db.query(ChatHistory).filter(
            ChatHistory.id == message_id,
            ChatHistory.user_id == user_data["user_id"]
        ).first()

        if not chat_message:
            raise HTTPException(status_code=404, detail="Message not found")

        # Delete the message
        db.delete(chat_message)
        db.commit()

        logger.info(f"Deleted chat message {message_id} for user {user_data['user_id']}")

        return {"message": "Chat message deleted successfully"}
    except Exception as e:
        logger.error(f"Error deleting chat message: {str(e)}")
        db.rollback()
        raise HTTPException(status_code=500, detail="Internal server error")


# Mount the router in the main application
def mount_chat_history_api(app):
    """
    Mount the chat history API routes to the main FastAPI app
    """
    app.include_router(router)