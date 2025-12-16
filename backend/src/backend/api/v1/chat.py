"""
Chat API v1 endpoints for persistent chat history
"""
from fastapi import APIRouter, Depends, HTTPException, Request
from typing import Optional
from uuid import UUID
import logging

from sqlalchemy.ext.asyncio import AsyncSession

from ...database.connection import get_async_db
from ...services.auth_validation import auth_validator
from ...services.chat_thread_service import get_chat_thread_service, ChatThreadService
from .schemas import (
    ChatThreadCreateRequest,
    ChatThreadUpdateRequest,
    ChatThreadResponse,
    ChatThreadListResponse,
    ChatMessageCreateRequest,
    ChatMessageResponse,
    ChatMessageListResponse,
    ErrorResponse
)

logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/api/v1/chat", tags=["chat-v1"])

@router.post("/threads", response_model=ChatThreadResponse, status_code=201)
async def create_chat_thread(
    request: Request,
    thread_data: ChatThreadCreateRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Create a new chat thread for authenticated users
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user_id = user_data.get("user_id")
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid user data")

    try:
        # Get chat thread service
        chat_service = get_chat_thread_service(db)

        # Create the chat thread
        thread = await chat_service.create_chat_thread(
            user_id=user_id,
            title=thread_data.title
        )

        return ChatThreadResponse(
            id=thread.id,
            user_id=thread.user_id,
            title=thread.title,
            created_at=thread.created_at,
            updated_at=thread.updated_at
        )
    except ValueError as e:
        logger.warning(f"Invalid input when creating chat thread for user {user_id}: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error creating chat thread for user {user_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/threads", response_model=ChatThreadListResponse)
async def get_user_chat_threads(
    request: Request,
    limit: int = 50,
    offset: int = 0,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Retrieve all chat threads for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user_id = user_data.get("user_id")
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid user data")

    try:
        # Get chat thread service
        chat_service = get_chat_thread_service(db)

        # Get user's chat threads
        threads, total_count = await chat_service.get_user_threads(
            user_id=user_id,
            limit=limit,
            offset=offset
        )

        # Convert to response format
        thread_responses = [
            ChatThreadResponse(
                id=thread.id,
                user_id=thread.user_id,
                title=thread.title,
                created_at=thread.created_at,
                updated_at=thread.updated_at
            )
            for thread in threads
        ]

        return ChatThreadListResponse(
            threads=thread_responses,
            total_count=total_count,
            limit=limit,
            offset=offset
        )
    except Exception as e:
        logger.error(f"Error retrieving chat threads for user {user_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/threads/{thread_id}", response_model=ChatThreadResponse)
async def get_chat_thread(
    request: Request,
    thread_id: UUID,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Retrieve a specific chat thread and its messages for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user_id = user_data.get("user_id")
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid user data")

    try:
        # Get chat thread service
        chat_service = get_chat_thread_service(db)

        # Get the specific thread
        thread = await chat_service.get_thread_by_id(thread_id, user_id)

        if not thread:
            raise HTTPException(status_code=404, detail="Chat thread not found or does not belong to user")

        return ChatThreadResponse(
            id=thread.id,
            user_id=thread.user_id,
            title=thread.title,
            created_at=thread.created_at,
            updated_at=thread.updated_at
        )
    except ValueError as e:
        logger.warning(f"Invalid input when retrieving chat thread {thread_id}: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error retrieving chat thread {thread_id} for user {user_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.put("/threads/{thread_id}", response_model=ChatThreadResponse)
async def update_chat_thread(
    request: Request,
    thread_id: UUID,
    thread_data: ChatThreadUpdateRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Update a chat thread title for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user_id = user_data.get("user_id")
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid user data")

    try:
        # Get chat thread service
        chat_service = get_chat_thread_service(db)

        # Update the thread title
        updated_thread = await chat_service.update_thread_title(
            thread_id=thread_id,
            user_id=user_id,
            title=thread_data.title
        )

        if not updated_thread:
            raise HTTPException(status_code=404, detail="Chat thread not found or does not belong to user")

        return ChatThreadResponse(
            id=updated_thread.id,
            user_id=updated_thread.user_id,
            title=updated_thread.title,
            created_at=updated_thread.created_at,
            updated_at=updated_thread.updated_at
        )
    except ValueError as e:
        logger.warning(f"Invalid input when updating chat thread {thread_id}: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error updating chat thread {thread_id} for user {user_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.delete("/threads/{thread_id}", status_code=204)
async def delete_chat_thread(
    request: Request,
    thread_id: UUID,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Delete a chat thread for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user_id = user_data.get("user_id")
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid user data")

    try:
        # Get chat thread service
        chat_service = get_chat_thread_service(db)

        # Delete the thread
        deleted = await chat_service.delete_thread(thread_id, user_id)

        if not deleted:
            raise HTTPException(status_code=404, detail="Chat thread not found or does not belong to user")

        return  # 204 No Content
    except Exception as e:
        logger.error(f"Error deleting chat thread {thread_id} for user {user_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.post("/threads/{thread_id}/messages", response_model=ChatMessageResponse, status_code=201)
async def add_message_to_thread(
    request: Request,
    thread_id: UUID,
    message_data: ChatMessageCreateRequest,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Add a message to a chat thread for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user_id = user_data.get("user_id")
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid user data")

    try:
        # Get chat thread service
        chat_service = get_chat_thread_service(db)

        # Add the message to the thread
        message = await chat_service.add_message_to_thread(
            thread_id=thread_id,
            user_id=user_id,
            role=message_data.role,
            content=message_data.content,
            sources=message_data.sources,
            selected_text_used=message_data.selected_text_used
        )

        # If this is the first message, auto-generate the thread title
        if len(message.thread.messages) == 1:  # This is the first message in the thread
            await chat_service.generate_thread_title_from_first_message(
                thread_id=thread_id,
                user_id=user_id,
                first_message_content=message_data.content
            )

        return ChatMessageResponse(
            id=message.id,
            thread_id=message.thread_id,
            role=message.role,
            content=message.content,
            sources=message.sources,
            selected_text_used=message.selected_text_used,
            created_at=message.created_at
        )
    except ValueError as e:
        logger.warning(f"Invalid input when adding message to thread {thread_id}: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error adding message to thread {thread_id} for user {user_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


@router.get("/threads/{thread_id}/messages", response_model=ChatMessageListResponse)
async def get_thread_messages(
    request: Request,
    thread_id: UUID,
    limit: int = 100,
    offset: int = 0,
    db: AsyncSession = Depends(get_async_db)
):
    """
    Get all messages in a specific chat thread for authenticated user
    """
    # Validate session
    user_data = await auth_validator.validate_session_from_request(request)
    if not user_data:
        raise HTTPException(status_code=401, detail="Unauthorized")

    user_id = user_data.get("user_id")
    if not user_id:
        raise HTTPException(status_code=401, detail="Invalid user data")

    try:
        # Get chat thread service
        chat_service = get_chat_thread_service(db)

        # Get messages from the thread
        messages, total_count = await chat_service.get_thread_messages(
            thread_id=thread_id,
            user_id=user_id,
            limit=limit,
            offset=offset
        )

        # Convert to response format
        message_responses = [
            ChatMessageResponse(
                id=message.id,
                thread_id=message.thread_id,
                role=message.role,
                content=message.content,
                sources=message.sources,
                selected_text_used=message.selected_text_used,
                created_at=message.created_at
            )
            for message in messages
        ]

        return ChatMessageListResponse(
            messages=message_responses,
            total_count=total_count,
            limit=limit,
            offset=offset
        )
    except ValueError as e:
        logger.warning(f"Invalid input when retrieving messages from thread {thread_id}: {str(e)}")
        raise HTTPException(status_code=400, detail=str(e))
    except Exception as e:
        logger.error(f"Error retrieving messages from thread {thread_id} for user {user_id}: {str(e)}")
        raise HTTPException(status_code=500, detail="Internal server error")


# Mount the router in the main application
def mount_chat_api(app):
    """
    Mount the chat API routes to the main FastAPI app
    """
    app.include_router(router)