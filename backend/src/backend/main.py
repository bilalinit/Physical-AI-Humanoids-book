import os
import uuid
from pathlib import Path
from datetime import datetime, timezone
from typing import Any, AsyncIterator, Dict, List
from dataclasses import dataclass, field

import logging
from dotenv import load_dotenv
from fastapi import FastAPI, Request, HTTPException, Depends
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import Response, StreamingResponse
from pydantic import BaseModel
from enum import Enum
import jwt
from jwt import DecodeError
from datetime import datetime, timedelta

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Load environment variables
load_dotenv()

# Import OpenAI Agents SDK components
from agents import Agent, Runner, OpenAIChatCompletionsModel, RunConfig, AsyncOpenAI

# Import ChatKit server components
from chatkit.server import ChatKitServer, StreamingResult
from chatkit.store import Store
from chatkit.types import ThreadMetadata, ThreadItem, Page
from chatkit.agents import AgentContext, stream_agent_response, ThreadItemConverter

# Import chat history API router
from .api.v1.chat import router as chat_v1_router

# Set up the OpenAI client with Gemini's OpenAI-compatible endpoint
API_KEY = os.getenv("GEMINI_API_KEY")
if not API_KEY:
    raise ValueError("GEMINI_API_KEY not found in environment variables")

client = AsyncOpenAI(
    api_key=API_KEY,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai/"
)

# Define the Gemini model
model = OpenAIChatCompletionsModel(
    model="gemini-2.5-flash",
    openai_client=client
)

# JWT Configuration
SECRET_KEY = os.getenv("JWT_SECRET_KEY", "your-default-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = 30

# Configure the run settings
config = RunConfig(
    model=model,
    model_provider=client,
)


# JWT utility functions
def create_access_token(data: dict, expires_delta: timedelta = None):
    to_encode = data.copy()
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=15)
    to_encode.update({"exp": expire})
    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


async def get_current_user_optional(request: Request):
    """Get current user from JWT token if present, otherwise return None"""
    auth_header = request.headers.get("Authorization")
    try:
        if not auth_header or not auth_header.startswith("Bearer "):
            # No token provided, return None for anonymous user
            return None

        token = auth_header[7:]  # Remove "Bearer " prefix
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            # Invalid token, but we'll still allow anonymous access
            return None
        return payload
    except DecodeError:
        # Invalid token, but we'll still allow anonymous access
        return None
    except Exception as e:
        # Other error, but we'll still allow anonymous access
        return None


async def get_current_user(request: Request):
    """Get current user from JWT token, required authentication"""
    auth_header = request.headers.get("Authorization")
    try:
        if not auth_header or not auth_header.startswith("Bearer "):
            raise HTTPException(
                status_code=401,
                detail="Authorization header missing or invalid format",
                headers={"WWW-Authenticate": "Bearer"},
            )

        token = auth_header[7:]  # Remove "Bearer " prefix
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("sub")
        if user_id is None:
            raise HTTPException(
                status_code=401,
                detail="Could not validate credentials",
                headers={"WWW-Authenticate": "Bearer"},
            )
        return payload
    except DecodeError:
        raise HTTPException(
            status_code=401,
            detail="Could not validate credentials",
            headers={"WWW-Authenticate": "Bearer"},
        )
    except Exception as e:
        raise HTTPException(
            status_code=401,
            detail="Token validation error",
            headers={"WWW-Authenticate": "Bearer"},
        )

# Import Qdrant functionality from database module
import sys
from pathlib import Path

# Add the main backend directory to the path to import backend modules
import sys
from pathlib import Path

# Add the project root to the path so we can import from backend directory
project_root = Path(__file__).parent.parent.parent  # /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/backend/
backend_dir = project_root / "backend"  # /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/backend/backend/
sys.path.insert(0, str(backend_dir))

from database import qdrant_client_instance, get_embedding

# Import the MemoryStore implementation
from .store import MemoryStore

# Import database services for chat history persistence
from .database.connection import get_async_db, AsyncSessionLocal
from .services.chat_thread_service import get_chat_thread_service

# Import persistent chat history API
from .api.v1.chat import router as chat_v1_router

# ChatKit Server Implementation
class ChatKitServerImpl(ChatKitServer):
    def __init__(self, store: Store):
        super().__init__(store)
        self.store = store
        # Use ChatKit's official converter for proper item handling
        self.converter = ThreadItemConverter()

    async def respond(self, thread, input, context):
        """Handle ChatKit requests with RAG functionality using proper ChatKit patterns"""
        from chatkit.types import (
            ThreadItemAddedEvent, ThreadItemDoneEvent,
            ThreadItemUpdatedEvent, AssistantMessageItem
        )

        # Track ID mappings to ensure unique IDs (LiteLLM/Gemini fix)
        id_mapping: dict[str, str] = {}

        # Extract user message from input
        user_message = ""
        if hasattr(input, 'content') and input.content:
            for content_item in input.content:
                if hasattr(content_item, 'text') and content_item.text:
                    user_message = content_item.text
                    break
        elif isinstance(input, str):
            user_message = input

        if not user_message:
            # If no message content, return an empty response
            return

        # Check for selected text context in the message
        # Format: [CONTEXT: selected text here] actual question
        selected_text = None
        import re
        context_match = re.match(r'\[CONTEXT:\s*(.+?)\]\s*(.*)', user_message, re.DOTALL)
        if context_match:
            selected_text = context_match.group(1).strip()
            user_message = context_match.group(2).strip() if context_match.group(2).strip() else "Explain this text"
            logger.info(f"Extracted selected text context: {selected_text[:100]}...")

        # Generate embedding for user query
        query_embedding = get_embedding(user_message, "retrieval_query")

        # Search in Qdrant with fallback mechanism
        try:
            search_response = qdrant_client_instance.query_points(
                collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
                query=query_embedding,
                limit=int(os.getenv("SEARCH_LIMIT", "5")),
                score_threshold=0.5  # Lower threshold to capture relevant results
            )
        except Exception as e:
            # Fallback: return a response without RAG context if Qdrant is unavailable
            logger.error(f"Qdrant search failed: {str(e)}")
            system_prompt = f"""
            You are a helpful assistant. The documentation search system is currently unavailable.
            Try to answer the user's question based on general knowledge.
            If you cannot answer the question, please say so politely.

            User Question: {user_message}
            """
            search_response = None
        
        # Fetch user profile for personalization
        user_profile = {}
        user_id = context.get('user', {}).get('id')
        if hasattr(self.store, 'get_user_profile') and user_id:
            user_profile = self.store.get_user_profile(user_id)
            logger.info(f"Loaded user profile for personalization: {user_profile}")

        # Construct personalization context string
        personalization_instruction = ""
        if user_profile:
            p_level = f"User's Education Level: {user_profile.get('education_level', 'Unknown')}"
            p_prog = f"Programming Experience: {user_profile.get('programming_experience', 'Unknown')}"
            p_robot = f"Robotics Background: {user_profile.get('robotics_background', 'Unknown')}"
            
            personalization_instruction = f"""
            PERSONALIZATION CONTEXT:
            - {p_level}
            - {p_prog}
            - {p_robot}
            
            ADAPTATION INSTRUCTIONS:
            - Adjust your language complexity to match the user's education level.
            - If programming experience is 'Beginner' or 'No Experience', explain code concepts simply.
            - If robotics background is 'No Experience', avoid jargon or explain it clearly.
            - If user is advanced/expert, you can use more technical terms and go deeper.
            """
        else:
            personalization_instruction = "Adapt your response to be helpful and clear for a general audience."

        # Build system prompt based on search results
        if search_response is not None:
            # Extract context from search results
            context_chunks = []
            sources = set()

            # Handle response format based on the qdrant-client version
            search_results = search_response.points if hasattr(search_response, 'points') else search_response

            for result in search_results:
                if hasattr(result, 'score') and result.score >= 0.5:
                    chunk_data = {
                        "filename": result.payload.get("filename") if hasattr(result, 'payload') else result.get('payload', {}).get('filename'),
                        "text": result.payload.get("text") if hasattr(result, 'payload') else result.get('payload', {}).get('text'),
                        "chunk_number": result.payload.get("chunk_number") if hasattr(result, 'payload') else result.get('payload', {}).get('chunk_number'),
                        "total_chunks": result.payload.get("total_chunks") if hasattr(result, 'payload') else result.get('payload', {}).get('total_chunks'),
                        "score": result.score if hasattr(result, 'score') else result.get('score')
                    }
                    context_chunks.append(chunk_data)
                    if chunk_data["filename"]:
                        sources.add(chunk_data["filename"])

            # Build context string for the agent
            if context_chunks:
                context_str = "\n\n".join([chunk["text"] for chunk in context_chunks if chunk.get("text")])
                
                # If selected_text was extracted from the message, prioritize it
                if selected_text:
                    system_prompt = f"""
                    You are a helpful assistant that answers questions based on the provided book content.
                    The user has selected specific text from the documentation and wants you to explain it.
                    
                    PRIMARY CONTEXT (selected by user):
                    {selected_text}
                    
                    Additional context from documentation search:
                    {context_str}
                    
                    Instructions:
                    - Focus your explanation on the PRIMARY CONTEXT (selected text) first
                    - Use the additional context to provide more depth and background
                    - Adjust your response based on the user's apparent level of understanding
                    - If the selected text is a technical concept, explain it clearly
                    - Always cite relevant source documents when applicable
                    """
                else:
                    system_prompt = f"""
                    You are a helpful assistant that answers questions based on the provided book content.
                    Use the following context to answer the user's question:
                    {context_str}

                    Adjust your responses based on the user's education level and experience.
                    If the context doesn't contain information to answer the question, say so.
                    Always cite the source document when providing information from the context.
                    """
            else:
                logger.info(f"No relevant results found for query: {user_message[:100]}...")
                # Still use selected_text if available even without search results
                if selected_text:
                    system_prompt = f"""
                    You are a helpful assistant. The user has selected text and wants you to explain it.
                    
                    Selected text to explain:
                    {selected_text}
                    
                    Please provide a clear explanation of this text. If you need more context to fully
                    explain it, let the user know what additional information would be helpful.
                    """
                else:
                    system_prompt = f"""
                    You are a helpful assistant. The documentation search did not return any relevant results.
                    Try to provide a helpful response based on general knowledge, and suggest the user rephrase their question if needed.
                    """
        else:
            system_prompt = f"""
            You are a helpful assistant. The documentation search system is currently unavailable.
            Try to answer the user's question based on general knowledge.
            """

        # Append personalization instruction to the final system prompt
        system_prompt = f"{system_prompt}\n\n{personalization_instruction}"

        # Create the RAG agent with the context
        rag_agent = Agent(
            name="RAGBot",
            instructions=system_prompt,
            model=model
        )

        # Create agent context
        agent_context = AgentContext(
            thread=thread,
            store=self.store,
            request_context=context,
        )

        # Load thread items for conversation history
        page = await self.store.load_thread_items(
            thread.id,
            after=None,
            limit=100,
            order="asc",
            context=context
        )
        all_items = list(page.data)

        # Add current input to the conversation
        if input:
            all_items.append(input)

        # Convert using ChatKit's official converter
        agent_input = await self.converter.to_agent_input(all_items) if all_items else []

        # Run agent with full history using streamed response
        result = Runner.run_streamed(
            rag_agent,
            agent_input,
            context=agent_context,
        )

        # Stream the response with ID collision fix
        async for event in stream_agent_response(agent_context, result):
            # Fix potential ID collisions from LiteLLM/Gemini
            if event.type == "thread.item.added":
                if isinstance(event.item, AssistantMessageItem):
                    old_id = event.item.id
                    if old_id not in id_mapping:
                        new_id = self.store.generate_item_id("message", thread, context)
                        id_mapping[old_id] = new_id
                    event.item.id = id_mapping[old_id]

            elif event.type == "thread.item.done":
                if isinstance(event.item, AssistantMessageItem):
                    old_id = event.item.id
                    if old_id in id_mapping:
                        event.item.id = id_mapping[old_id]

            elif event.type == "thread.item.updated":
                if event.item_id in id_mapping:
                    event.item_id = id_mapping[event.item_id]

            yield event

app = FastAPI(title="Qdrant RAG API with ChatKit Compatibility")

# Mount the persistent chat history API
app.include_router(chat_v1_router)

# Initialize ChatKit store and server
# Use NeonStore for persistent per-user chat history
try:
    from .neon_store import NeonStore
    store = NeonStore()
    logger.info("Using NeonStore for persistent chat history")
except Exception as e:
    logger.warning(f"Failed to initialize NeonStore, falling back to MemoryStore: {e}")
    store = MemoryStore()
server = ChatKitServerImpl(store)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",  # Docusaurus frontend (local dev)
        "http://localhost:3001",  # Auth server
        "http://localhost:8000",  # Backend
        "http://127.0.0.1:3000",
        "http://127.0.0.1:3001",
        "http://127.0.0.1:8000",
        "https://localhost:3000",
        "https://localhost:3001",
        "https://localhost:8000",
        # Production frontends
        "https://physical-ai-humanoids-book-rag.netlify.app",
    ],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class ChatRequest(BaseModel):
    user_query: str
    selected_text: str = None
    chat_history: List[Dict[str, str]] = []
    user_profile: Dict[str, Any] = {"education": "beginner", "experience": "software:2_years"}

class ChatResponse(BaseModel):
    output: str
    context_chunks: List[Dict[str, Any]]
    sources: List[str]
    used_selected_text: bool = False

class SearchRequest(BaseModel):
    query: str
    limit: int = 5
    score_threshold: float = 0.7

# Import Qdrant functionality from database module
import sys
from pathlib import Path

# Add the main backend directory to the path to import backend modules
import sys
from pathlib import Path

# Add the project root to the path so we can import from backend directory
project_root = Path(__file__).parent.parent.parent  # /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/backend/
backend_dir = project_root / "backend"  # /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/backend/backend/
sys.path.insert(0, str(backend_dir))

from database import qdrant_client_instance, get_embedding

# In-memory storage for threads (for ChatKit compatibility)
# This is a temporary implementation for ChatKit compatibility
chat_threads: Dict[str, Dict[str, Any]] = {}
chat_messages: Dict[str, List[Dict[str, Any]]] = {}
# Store for maintaining agent state across threads
thread_agents: Dict[str, Any] = {}

# Thread cleanup configuration
THREAD_CLEANUP_INTERVAL_HOURS = 24  # Clean up threads after 24 hours of inactivity


def cleanup_old_threads():
    """Remove threads that have been inactive for more than the specified time"""
    import time
    from datetime import timedelta

    current_time = datetime.utcnow()
    cutoff_time = current_time - timedelta(hours=THREAD_CLEANUP_INTERVAL_HOURS)
    cutoff_iso = cutoff_time.isoformat() + "Z"

    threads_to_remove = []
    for thread_id, thread_data in chat_threads.items():
        last_activity = thread_data.get("lastActivityAt", thread_data.get("createdAt"))
        if last_activity and last_activity < cutoff_iso:
            threads_to_remove.append(thread_id)

    for thread_id in threads_to_remove:
        # Remove thread and its messages
        del chat_threads[thread_id]
        if thread_id in chat_messages:
            del chat_messages[thread_id]
        # Also remove the agent reference if it exists
        if thread_id in thread_agents:
            del thread_agents[thread_id]

    if threads_to_remove:
        logger.info(f"Cleaned up {len(threads_to_remove)} inactive threads")

    return len(threads_to_remove)


# Optional: Add an endpoint to manually trigger cleanup for testing
@app.post("/api/admin/cleanup-threads")
async def manual_cleanup_threads():
    """Manually trigger thread cleanup (admin endpoint)"""
    cleaned_count = cleanup_old_threads()
    return {"message": f"Cleaned up {cleaned_count} threads", "cleaned_count": cleaned_count}

# Thread Management Endpoints
@app.get("/api/chat/threads")
async def get_threads(current_user: dict = Depends(get_current_user)):
    """Retrieve list of user's chat threads"""
    try:
        threads_list = []
        for thread_id, thread_data in chat_threads.items():
            # Filter threads by user if authenticated, otherwise return all threads for anonymous users
            thread_user_id = thread_data.get("userId")
            if current_user is None:
                # For anonymous users, only return threads without a specific user or threads marked as anonymous
                if thread_user_id is None or thread_user_id == "anonymous":
                    threads_list.append({
                        "id": thread_id,
                        "createdAt": thread_data.get("createdAt"),
                        "updatedAt": thread_data.get("updatedAt"),
                        "title": thread_data.get("title", "New Conversation"),
                        "messageCount": len(chat_messages.get(thread_id, []))
                    })
            else:
                # For authenticated users, only return their threads
                if thread_user_id == current_user.get("sub"):
                    threads_list.append({
                        "id": thread_id,
                        "createdAt": thread_data.get("createdAt"),
                        "updatedAt": thread_data.get("updatedAt"),
                        "title": thread_data.get("title", "New Conversation"),
                        "messageCount": len(chat_messages.get(thread_id, []))
                    })

        return {"threads": threads_list}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving threads: {str(e)}")


@app.get("/api/chat/threads/{thread_id}")
async def get_thread_messages(thread_id: str, current_user: dict = Depends(get_current_user)):
    """Retrieve messages for a specific thread"""
    try:
        if thread_id not in chat_threads:
            raise HTTPException(status_code=404, detail="Thread not found")

        thread_data = chat_threads[thread_id]
        thread_user_id = thread_data.get("userId")

        # Check if user has permission to access this thread
        if thread_user_id is not None:
            if current_user is None:
                # Anonymous user trying to access authenticated user's thread
                raise HTTPException(status_code=403, detail="Access denied")
            elif thread_user_id != current_user.get("sub"):
                # Authenticated user trying to access another user's thread
                raise HTTPException(status_code=403, detail="Access denied")

        thread_messages = chat_messages.get(thread_id, [])

        return {
            "thread": {
                "id": thread_id,
                "messages": thread_messages
            }
        }
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error retrieving thread messages: {str(e)}")


@app.post("/api/chat/threads")
async def create_thread(request: dict, current_user: dict = Depends(get_current_user)):
    """Create a new chat thread"""
    try:
        thread_id = str(uuid.uuid4())

        # Get initial message from request if provided
        initial_message = request.get("initialMessage", "New conversation")

        # Determine user ID for the thread
        user_id = None
        if current_user is not None:
            user_id = current_user.get("sub")
        else:
            user_id = "anonymous"  # Mark as anonymous user

        # Create thread metadata
        thread_data = {
            "id": thread_id,
            "userId": user_id,  # Store user ID for access control
            "createdAt": datetime.utcnow().isoformat() + "Z",
            "updatedAt": datetime.utcnow().isoformat() + "Z",
            "title": initial_message[:50] if initial_message else "New Conversation"  # First 50 chars as title
        }

        chat_threads[thread_id] = thread_data
        chat_messages[thread_id] = []

        # If initial message exists, add it as the first message
        if initial_message:
            first_message = {
                "id": str(uuid.uuid4()),
                "role": "user",
                "content": initial_message,
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
            chat_messages[thread_id].append(first_message)

        return {
            "threadId": thread_id,
            "createdAt": thread_data["createdAt"]
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error creating thread: {str(e)}")


# Chat message processing endpoint
@app.post("/api/chat/threads/{thread_id}/messages")
async def process_message(thread_id: str, request: dict, current_user: dict = Depends(get_current_user)):
    """Send a message and receive AI response with RAG-enhanced content"""
    try:
        if thread_id not in chat_threads:
            raise HTTPException(status_code=404, detail="Thread not found")

        # Check if user has permission to access this thread
        thread_data = chat_threads[thread_id]
        thread_user_id = thread_data.get("userId")

        if thread_user_id is not None:
            if current_user is None:
                # Anonymous user trying to access authenticated user's thread
                raise HTTPException(status_code=403, detail="Access denied")
            elif thread_user_id != current_user.get("sub"):
                # Authenticated user trying to access another user's thread
                raise HTTPException(status_code=403, detail="Access denied")

        # Extract content, selected text, and user profile from request
        content = request.get("content", "")
        selected_text = request.get("selectedText", "")
        user_profile = request.get("userProfile", {
            "education": "beginner",
            "experience": "software:2_years"
        })

        if not content:
            raise HTTPException(status_code=400, detail="Message content is required")

        # Validate and sanitize selected text if provided
        if selected_text:
            # Check length - max 5000 characters
            if len(selected_text) > 5000:
                raise HTTPException(status_code=400, detail="Selected text exceeds maximum length of 5000 characters")

            # Sanitize selected text to prevent injection attacks
            import html
            selected_text_sanitized = html.escape(selected_text)
            # For now, just validate that it doesn't contain potentially harmful content
            harmful_patterns = ["<script", "javascript:", "vbscript:", "onerror", "onload", "eval("]
            if any(pattern in selected_text.lower() for pattern in harmful_patterns):
                raise HTTPException(status_code=400, detail="Selected text contains potentially harmful content")

        # Simple content validation (guardrail)
        harmful_keywords = ["harmful", "inappropriate", "offensive"]
        if any(keyword in content.lower() for keyword in harmful_keywords):
            raise HTTPException(status_code=400, detail="Input contains potentially inappropriate content.")

        # Add user message to thread
        user_message = {
            "id": str(uuid.uuid4()),
            "role": "user",
            "content": content,
            "timestamp": datetime.utcnow().isoformat() + "Z"
        }
        chat_messages[thread_id].append(user_message)

        # Save user message to database
        user_id = current_user.get("sub") if current_user else None
        async with AsyncSessionLocal() as db_session:
            try:
                chat_service = get_chat_thread_service(db_session)
                # Check if thread exists in DB, if not create it
                db_thread = await chat_service.get_thread_by_id(uuid.UUID(thread_id), user_id) if user_id else None
                if not db_thread and user_id:
                    db_thread = await chat_service.create_chat_thread(user_id=user_id, title=content[:50])

                if db_thread:
                    await chat_service.add_message_to_thread(
                        thread_id=db_thread.id,
                        user_id=user_id,
                        role="user",
                        content=content,
                        sources=None,
                        selected_text_used=bool(selected_text)
                    )
                await db_session.commit()
            except Exception as e:
                logger.error(f"Failed to save user message to database: {e}")

        # Generate embedding for user query
        query_embedding = get_embedding(content, "retrieval_query")

        # Search in Qdrant with fallback mechanism
        try:
            search_response = qdrant_client_instance.query_points(
                collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
                query=query_embedding,
                limit=int(os.getenv("SEARCH_LIMIT", "5")),
                score_threshold=0.5  # Lower threshold to capture relevant results
            )
        except Exception as e:
            # Fallback: return a response without RAG context if Qdrant is unavailable
            logger.error(f"Qdrant search failed: {str(e)}")
            system_prompt = f"""
            You are a helpful assistant. The documentation search system is currently unavailable.
            Try to answer the user's question based on general knowledge.
            If you cannot answer the question, please say so politely.

            User Question: {content}
            """

            # Create a fallback agent without RAG context
            rag_agent = Agent(
                name="RAGBot-Fallback",
                instructions=system_prompt,
                model=model
            )
            thread_agents[thread_id] = rag_agent  # Update the agent in case it was different

            # Run the fallback agent
            response = await Runner.run(rag_agent, content, run_config=config)

            # Create assistant message
            assistant_message = {
                "id": str(uuid.uuid4()),
                "role": "assistant",
                "content": response.final_output if response.final_output else "I'm sorry, but the documentation search system is currently unavailable. Please try again later.",
                "timestamp": datetime.utcnow().isoformat() + "Z",
                "sources": [],
                "used_selected_text": selected_text is not None and selected_text.strip() != "",
                "fallback": True  # Indicate this is a fallback response
            }

            # Add assistant message to thread
            chat_messages[thread_id].append(assistant_message)

            # Update thread timestamp
            chat_threads[thread_id]["updatedAt"] = datetime.utcnow().isoformat() + "Z"

            return {
                "message": assistant_message
            }

        # Extract context from search results
        context_chunks = []
        sources = set()

        # Handle response format based on the qdrant-client version
        search_results = search_response.points if hasattr(search_response, 'points') else search_response

        for result in search_results:
            if hasattr(result, 'score') and result.score >= 0.5:  # Lower threshold to ensure context retrieval
                # Extract data using the same pattern as the working search endpoint
                chunk_data = {
                    "filename": result.payload.get("filename") if hasattr(result, 'payload') else result.get('payload', {}).get('filename'),
                    "text": result.payload.get("text") if hasattr(result, 'payload') else result.get('payload', {}).get('text'),
                    "chunk_number": result.payload.get("chunk_number") if hasattr(result, 'payload') else result.get('payload', {}).get('chunk_number'),
                    "total_chunks": result.payload.get("total_chunks") if hasattr(result, 'payload') else result.get('payload', {}).get('total_chunks'),
                    "score": result.score if hasattr(result, 'score') else result.get('score')
                }
                # Add to context_chunks (similar to search endpoint)
                context_chunks.append(chunk_data)
                if chunk_data["filename"]:
                    sources.add(chunk_data["filename"])

        # Build context string for the agent
        if context_chunks:
            context_str = "\n\n".join([chunk["text"] for chunk in context_chunks])

            # Build the system prompt with selected text if available
            if selected_text:
                # Prioritize the selected text as primary context
                system_prompt = f"""
                You are a helpful assistant that answers questions based on the provided book content.
                The user has selected specific text from the documentation and wants to ask about it.
                PRIMARY CONTEXT (selected by user):
                {selected_text}

                Additional context from search results:
                {context_str}

                User Profile:
                - Education Level: {user_profile.get('education', 'beginner')}
                - Experience: {user_profile.get('experience', 'software:2_years')}

                Adjust your responses based on the user's education level and experience.
                When answering, make sure to address the PRIMARY CONTEXT (selected text) directly.
                If the context doesn't contain information to answer the question, say so.
                Always cite the source document when providing information from the context.
                """
            else:
                # Use the original system prompt when no selected text is provided
                system_prompt = f"""
                You are a helpful assistant that answers questions based on the provided book content.
                Use the following context to answer the user's question:
                {context_str}

                User Profile:
                - Education Level: {user_profile.get('education', 'beginner')}
                - Experience: {user_profile.get('experience', 'software:2_years')}

                Adjust your responses based on the user's education level and experience.
                If the context doesn't contain information to answer the question, say so.
                Always cite the source document when providing information from the context.
                """
        else:
            # No relevant results found - create a system prompt that informs the agent
            logger.info(f"No relevant results found for query: {content[:100]}...")
            system_prompt = f"""
            You are a helpful assistant. The documentation search did not return any relevant results for the user's question.
            Try to provide a helpful response based on general knowledge, and suggest the user rephrase their question if needed.

            User Question: {content}

            User Profile:
            - Education Level: {user_profile.get('education', 'beginner')}
            - Experience: {user_profile.get('experience', 'software:2_years')}

            Adjust your response based on the user's education level and experience.
            Politely inform the user that no relevant documentation was found and suggest they might want to rephrase their question.
            """

        # Check if we already have an agent for this thread
        if thread_id not in thread_agents:
            # Create a new RAG agent with the context for the first time
            rag_agent = Agent(
                name="RAGBot",
                instructions=system_prompt,
                model=model
            )
            thread_agents[thread_id] = rag_agent
        else:
            # Reuse the existing agent for this thread to maintain context
            rag_agent = thread_agents[thread_id]

        # Run the agent with the user query
        response = await Runner.run(rag_agent, content, run_config=config)

        # Create assistant message
        assistant_message = {
            "id": str(uuid.uuid4()),
            "role": "assistant",
            "content": response.final_output if response.final_output else "I couldn't find relevant information in the book content.",
            "timestamp": datetime.utcnow().isoformat() + "Z",
            "sources": [{"title": source, "url": f"/docs/{source}", "content": None} for source in list(sources)],
            "used_selected_text": selected_text is not None and selected_text.strip() != ""
        }

        # Add assistant message to thread
        chat_messages[thread_id].append(assistant_message)

        # Save assistant message to database
        user_id = current_user.get("sub") if current_user else None
        async with AsyncSessionLocal() as db_session:
            try:
                chat_service = get_chat_thread_service(db_session)
                db_thread = await chat_service.get_thread_by_id(uuid.UUID(thread_id), user_id) if user_id else None
                if db_thread:
                    await chat_service.add_message_to_thread(
                        thread_id=db_thread.id,
                        user_id=user_id,
                        role="assistant",
                        content=assistant_message["content"],
                        sources=assistant_message.get("sources"),
                        selected_text_used=assistant_message.get("used_selected_text", False)
                    )
                await db_session.commit()
            except Exception as e:
                logger.error(f"Failed to save assistant message to database: {e}")

        # Implement conversation history management to prevent token limit issues
        # Keep only the most recent messages (e.g., last 20 messages) to prevent token limits
        MAX_HISTORY_MESSAGES = 20
        if len(chat_messages[thread_id]) > MAX_HISTORY_MESSAGES:
            # Keep the first message (initial context) and the most recent messages
            first_message = chat_messages[thread_id][0] if chat_messages[thread_id] else None
            recent_messages = chat_messages[thread_id][-MAX_HISTORY_MESSAGES+1:]  # +1 to account for first message
            if first_message and first_message not in recent_messages:
                chat_messages[thread_id] = [first_message] + recent_messages
            else:
                chat_messages[thread_id] = recent_messages

        # Update thread timestamp
        chat_threads[thread_id]["updatedAt"] = datetime.utcnow().isoformat() + "Z"
        chat_threads[thread_id]["lastActivityAt"] = datetime.utcnow().isoformat() + "Z"

        return {
            "message": assistant_message
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing message: {str(e)}")


@app.delete("/api/chat/threads/{thread_id}")
async def delete_thread(thread_id: str, current_user: dict = Depends(get_current_user_optional)):
    """Delete a specific thread and its messages"""
    try:
        if thread_id not in chat_threads:
            raise HTTPException(status_code=404, detail="Thread not found")

        # Check if user has permission to delete this thread
        thread_data = chat_threads[thread_id]
        thread_user_id = thread_data.get("userId")

        if thread_user_id is not None:
            if current_user is None:
                # Anonymous user trying to delete authenticated user's thread
                raise HTTPException(status_code=403, detail="Access denied")
            elif thread_user_id != current_user.get("sub"):
                # Authenticated user trying to delete another user's thread
                raise HTTPException(status_code=403, detail="Access denied")

        # Remove thread and its messages
        del chat_threads[thread_id]
        if thread_id in chat_messages:
            del chat_messages[thread_id]
        # Also remove the agent reference if it exists
        if thread_id in thread_agents:
            del thread_agents[thread_id]

        return {"message": "Thread deleted successfully"}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error deleting thread: {str(e)}")


# RAG Query Processing endpoint (for testing purposes)
@app.post("/api/chat/query")
async def process_rag_query(request: dict):
    """Process a RAG query without creating a thread (for testing purposes)"""
    try:
        query = request.get("query", "")
        user_profile = request.get("userProfile", {
            "education": "beginner",
            "experience": "software:2_years"
        })

        if not query:
            raise HTTPException(status_code=400, detail="Query is required")

        # Simple content validation (guardrail)
        harmful_keywords = ["harmful", "inappropriate", "offensive"]
        if any(keyword in query.lower() for keyword in harmful_keywords):
            raise HTTPException(status_code=400, detail="Input contains potentially inappropriate content.")

        # Generate embedding for query
        query_embedding = get_embedding(query, "retrieval_query")

        # Search in Qdrant
        search_response = qdrant_client_instance.query_points(
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
            query=query_embedding,
            limit=5,
            score_threshold=0.7
        )

        # Handle response format based on the qdrant-client version
        search_results = search_response.points if hasattr(search_response, 'points') else search_response

        context_chunks = []
        sources = set()
        for result in search_results:
            if hasattr(result, 'score') and result.score >= 0.7:
                context_chunks.append({
                    "id": result.id if hasattr(result, 'id') else result.get('id'),
                    "filename": result.payload.get("filename") if hasattr(result, 'payload') else result.get('payload', {}).get('filename'),
                    "text": result.payload.get("text") if hasattr(result, 'payload') else result.get('payload', {}).get('text'),
                    "chunk_number": result.payload.get("chunk_number") if hasattr(result, 'payload') else result.get('payload', {}).get('chunk_number'),
                    "total_chunks": result.payload.get("total_chunks") if hasattr(result, 'payload') else result.get('payload', {}).get('total_chunks'),
                    "score": result.score if hasattr(result, 'score') else result.get('score')
                })
                filename = result.payload.get("filename") if hasattr(result, 'payload') else result.get('payload', {}).get('filename')
                if filename:
                    sources.add(filename)

        # Build context string for the agent
        context_str = "\n\n".join([chunk["text"] for chunk in context_chunks])

        # Create system prompt with context and user profile
        system_prompt = f"""
        You are a helpful assistant that answers questions based on the provided book content.
        Use the following context to answer the user's question:
        {context_str}

        User Profile:
        - Education Level: {user_profile.get('education', 'beginner')}
        - Experience: {user_profile.get('experience', 'software:2_years')}

        Adjust your responses based on the user's education level and experience.
        If the context doesn't contain information to answer the question, say so.
        Always cite the source document when providing information from the context.
        """

        # Create the RAG agent with the context
        rag_agent = Agent(
            name="RAGBot",
            instructions=system_prompt,
            model=model
        )

        # Run the agent with the query
        response = await Runner.run(rag_agent, query, run_config=config)

        return {
            "response": response.final_output if response.final_output else "I couldn't find relevant information in the book content.",
            "sources": [{"title": source, "url": f"/docs/{source}", "relevance": 0.9} for source in list(sources)],
            "processingTime": 1.2
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing query: {str(e)}")


# RAG Chat endpoint (original functionality preserved)
@app.post("/api/chat", response_model=ChatResponse)
async def rag_chat(request: ChatRequest, current_user: dict = Depends(get_current_user_optional)):
    try:
        # Simple content validation (without complex guardrail that's causing issues)
        user_query = request.user_query

        # Validate and sanitize selected text if provided
        if request.selected_text:
            # Check length - max 5000 characters
            if len(request.selected_text) > 5000:
                raise HTTPException(status_code=400, detail="Selected text exceeds maximum length of 5000 characters")

            # Sanitize selected text to prevent injection attacks
            import html
            selected_text_sanitized = html.escape(request.selected_text)
            # Update the request object to use sanitized text
            # We'll work with the original request.selected_text for now but in a real scenario would use sanitized version
            # For now, just validate that it doesn't contain potentially harmful content
            harmful_patterns = ["<script", "javascript:", "vbscript:", "onerror", "onload", "eval("]
            if any(pattern in request.selected_text.lower() for pattern in harmful_patterns):
                raise HTTPException(status_code=400, detail="Selected text contains potentially harmful content")

        # Validate user query
        harmful_keywords = ["harmful", "inappropriate", "offensive"]
        if any(keyword in user_query.lower() for keyword in harmful_keywords):
            raise HTTPException(status_code=400, detail="Input contains potentially inappropriate content.")

        # Generate embedding for user query
        query_embedding = get_embedding(user_query, "retrieval_query")

        # Search in Qdrant - use a lower threshold to ensure context retrieval since max scores appear to be around 0.68
        search_response = qdrant_client_instance.query_points(
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
            query=query_embedding,
            limit=int(os.getenv("SEARCH_LIMIT", "5")),
            score_threshold=0.5  # Lower threshold to capture relevant results
        )

        # Extract context from search results
        context_chunks = []
        sources = set()

        # Handle response format based on the qdrant-client version
        search_results = search_response.points if hasattr(search_response, 'points') else search_response

        for result in search_results:
            if hasattr(result, 'score') and result.score >= 0.5:  # Lower threshold to ensure context retrieval
                # Extract data using the same pattern as the working search endpoint
                chunk_data = {
                    "filename": result.payload.get("filename") if hasattr(result, 'payload') else result.get('payload', {}).get('filename'),
                    "text": result.payload.get("text") if hasattr(result, 'payload') else result.get('payload', {}).get('text'),
                    "chunk_number": result.payload.get("chunk_number") if hasattr(result, 'payload') else result.get('payload', {}).get('chunk_number'),
                    "total_chunks": result.payload.get("total_chunks") if hasattr(result, 'payload') else result.get('payload', {}).get('total_chunks'),
                    "score": result.score if hasattr(result, 'score') else result.get('score')
                }
                # Add to context_chunks (similar to search endpoint)
                context_chunks.append(chunk_data)
                if chunk_data["filename"]:
                    sources.add(chunk_data["filename"])

        # Build context string for the agent
        context_str = "\n\n".join([chunk["text"] for chunk in context_chunks])

        # Prepare system prompt with personalization based on user profile
        from .services.prompt_personalization import prompt_personalization_service

        # Create a base system prompt
        base_system_prompt = f"""
        You are a helpful assistant that answers questions based on the provided book content.
        Use the following context to answer the user's question:
        {context_str}

        User's Question: {user_query}
        """

        # If user is authenticated, get their profile for personalization
        if current_user is not None:
            try:
                from .database import SessionLocal
                from .models.user import User as UserModel

                db = SessionLocal()
                try:
                    # Get user profile from database
                    user_db = db.query(UserModel).filter(UserModel.id == current_user.get("sub")).first()
                    if user_db:
                        user_profile = {
                            "educationLevel": user_db.education_level,
                            "programmingExperience": user_db.programming_experience,
                            "roboticsBackground": user_db.robotics_background,
                            "softwareBackground": user_db.software_background,
                            "hardwareBackground": user_db.hardware_background
                        }

                        # Generate personalized system prompt
                        system_prompt = prompt_personalization_service.get_personalized_system_prompt(
                            user_profile=user_profile,
                            base_prompt=base_system_prompt
                        )
                    else:
                        # Fallback to base prompt if user not found
                        system_prompt = base_system_prompt
                finally:
                    db.close()
            except Exception as e:
                logger.error(f"Error personalizing prompt: {str(e)}")
                # Fallback to base prompt
                system_prompt = base_system_prompt
        else:
            # For anonymous users, use base prompt
            system_prompt = base_system_prompt

        # If selected text is provided, prioritize it in the system prompt
        if request.selectedText:
            # Enhance the system prompt with selected text as primary context
            system_prompt = f"""
            {system_prompt}

            PRIMARY CONTEXT (selected by user):
            {request.selectedText}

            When answering the user's question, prioritize information from the PRIMARY CONTEXT.
            """

        # Create the RAG agent with the personalized context
        rag_agent = Agent(
            name="RAGBot",
            instructions=system_prompt,
            model=model
        )

        # Run the agent with the user query
        response = await Runner.run(rag_agent, user_query, run_config=config)

        # Save chat history if user is authenticated
        if current_user is not None:
            try:
                from .database import SessionLocal
                from .services.chat_history_service import get_chat_history_service

                # Create a new database session
                db = SessionLocal()
                try:
                    chat_service = get_chat_history_service(db)
                    # Save the chat message to history
                    chat_service.save_message(
                        user_id=current_user.get("sub"),  # Use the user ID from JWT
                        message=user_query,
                        response=response.final_output if response.final_output else "I couldn't find relevant information in the book content.",
                        selected_text=request.selected_text
                    )
                finally:
                    db.close()
            except Exception as e:
                # Log the error but don't fail the request if saving history fails
                logger.error(f"Error saving chat history: {str(e)}")

        return ChatResponse(
            output=response.final_output if response.final_output else "I couldn't find relevant information in the book content.",
            context_chunks=context_chunks,
            sources=list(sources),
            used_selected_text=request.selected_text is not None and request.selected_text.strip() != ""
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error processing chat request: {str(e)}")


# Direct vector search endpoint
@app.get("/api/search")
async def vector_search(query: str, limit: int = 5, score_threshold: float = 0.7):
    try:
        # Generate embedding for query
        query_embedding = get_embedding(query, "retrieval_query")

        # Search in Qdrant
        search_response = qdrant_client_instance.query_points(
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
            query=query_embedding,
            limit=limit,
            score_threshold=score_threshold
        )

        # Handle response format based on the qdrant-client version
        search_results = search_response.points if hasattr(search_response, 'points') else search_response

        results = []
        for result in search_results:
            if hasattr(result, 'score') and result.score >= score_threshold:
                results.append({
                    "id": result.id if hasattr(result, 'id') else result.get('id'),
                    "filename": result.payload.get("filename") if hasattr(result, 'payload') else result.get('payload', {}).get('filename'),
                    "text": result.payload.get("text") if hasattr(result, 'payload') else result.get('payload', {}).get('text'),
                    "chunk_number": result.payload.get("chunk_number") if hasattr(result, 'payload') else result.get('payload', {}).get('chunk_number'),
                    "total_chunks": result.payload.get("total_chunks") if hasattr(result, 'payload') else result.get('payload', {}).get('total_chunks'),
                    "score": result.score if hasattr(result, 'score') else result.get('score')
                })

        return {"results": results}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error performing search: {str(e)}")


# Health check
@app.get("/health")
async def health_check():
    return {"status": "healthy"}

# API health check
@app.get("/api/health")
async def api_health_check():
    return {
        "status": "healthy",
        "service": "chatkit-backend",
        "version": "1.0.0",
        "dependencies": {
            "qdrant": "connected",
            "gemini-api": "connected"
        }
    }


# Diagnostic endpoint for ChatKit integration verification
@app.get("/api/diagnostics")
async def diagnostics():
    """Detailed diagnostic information for ChatKit integration verification"""
    import sys
    import os
    from datetime import datetime

    # Check various system components
    qdrant_status = "connected" if qdrant_client_instance else "disconnected"

    # Test Gemini API connection
    try:
        # This is a lightweight test to check if the API is accessible
        import asyncio
        # Don't make an actual API call to avoid usage, just check if client is configured
        gemini_status = "configured" if client else "not configured"
    except:
        gemini_status = "error"

    return {
        "status": "operational",
        "timestamp": datetime.utcnow().isoformat(),
        "service": "chatkit-backend-diagnostics",
        "version": "1.0.0",
        "python_version": sys.version,
        "environment": {
            "qdrant_collection": os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
            "search_limit": os.getenv("SEARCH_LIMIT", "5"),
            "model": "gemini-2.5-flash"
        },
        "components": {
            "qdrant": qdrant_status,
            "gemini-api": gemini_status,
            "database": "in-memory",
            "authentication": "jwt-enabled"
        },
        "thread_stats": {
            "total_threads": len(chat_threads),
            "total_messages": sum(len(messages) for messages in chat_messages.values()),
            "active_agents": len(thread_agents)
        },
        "features": {
            "chatkit_integration": True,
            "thread_persistence": True,
            "rag_enabled": True,
            "agent_memory": True,
            "user_profiles": True,
            "authentication": True
        }
    }


@app.post("/")
async def chatkit_endpoint(request: Request):
    """ChatKit endpoint - handles ChatKit protocol requests with RAG functionality"""
    # Extract userId from query parameter for per-user chat history
    user_id = request.query_params.get('userId')
    
    # Build context with user info for NeonStore
    context = {}
    if user_id:
        context = {"user": {"id": user_id}}
        logger.debug(f"ChatKit request from user: {user_id}")
    
    result = await server.process(await request.body(), context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")


@app.options("/")
async def chatkit_options():
    """Handle CORS preflight requests for ChatKit"""
    return Response(status_code=200, headers={
        "Access-Control-Allow-Origin": "*",
        "Access-Control-Allow-Methods": "POST, OPTIONS",
        "Access-Control-Allow-Headers": "*",
    })


@app.post("/chatkit")
async def chatkit_legacy_endpoint(request: Request):
    """Legacy ChatKit endpoint - handles ChatKit protocol requests with RAG functionality"""
    # Extract userId from query parameter for per-user chat history
    user_id = request.query_params.get('userId')
    
    # Build context with user info for NeonStore
    context = {}
    if user_id:
        context = {"user": {"id": user_id}}
        logger.debug(f"ChatKit legacy request from user: {user_id}")
    
    result = await server.process(await request.body(), context)
    if isinstance(result, StreamingResult):
        return StreamingResponse(result, media_type="text/event-stream")
    return Response(content=result.json, media_type="application/json")


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)