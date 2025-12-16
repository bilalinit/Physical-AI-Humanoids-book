"""
Pydantic schemas for chat API v1
"""
from pydantic import BaseModel
from typing import List, Optional, Dict, Any
from datetime import datetime
from uuid import UUID


# Chat Thread Schemas
class ChatThreadCreateRequest(BaseModel):
    title: Optional[str] = None


class ChatThreadUpdateRequest(BaseModel):
    title: str


class ChatThreadResponse(BaseModel):
    id: UUID
    user_id: str
    title: str
    created_at: datetime
    updated_at: datetime

    class Config:
        from_attributes = True


class ChatThreadListResponse(BaseModel):
    threads: List[ChatThreadResponse]
    total_count: int
    limit: int
    offset: int


# Chat Message Schemas
class ChatMessageCreateRequest(BaseModel):
    role: str
    content: str
    sources: Optional[Dict[str, Any]] = None
    selected_text_used: bool = False


class ChatMessageResponse(BaseModel):
    id: UUID
    thread_id: UUID
    role: str
    content: str
    sources: Optional[Dict[str, Any]]
    selected_text_used: bool
    created_at: datetime

    class Config:
        from_attributes = True


class ChatMessageListResponse(BaseModel):
    messages: List[ChatMessageResponse]
    total_count: int
    limit: int
    offset: int


# API Response Schemas
class SuccessResponse(BaseModel):
    success: bool
    message: str


class ErrorResponse(BaseModel):
    error: Dict[str, str]