"""
NeonStore - PostgreSQL-backed store for ChatKit that implements per-user chat history
Uses the existing chat_threads and chat_messages tables in Neon PostgreSQL
"""
import os
import uuid
import json
import logging
from datetime import datetime, timezone
from typing import Any, Optional

import psycopg2
from psycopg2.extras import RealDictCursor
from dotenv import load_dotenv

# Import ChatKit types
try:
    from chatkit.store import Store
    from chatkit.types import ThreadMetadata, ThreadItem, Page
    from chatkit.types import AssistantMessageItem, UserMessageItem
    CHATKIT_AVAILABLE = True
except ImportError:
    CHATKIT_AVAILABLE = False
    # Fallback types if chatkit not available
    from dataclasses import dataclass, field
    from typing import TypeVar, Generic, List, Dict
    
    T = TypeVar('T')
    
    @dataclass
    class ThreadMetadata:
        id: str
        created_at: datetime
        metadata: Dict[str, Any] = field(default_factory=dict)
    
    @dataclass
    class Page(Generic[T]):
        data: List[T]
        has_more: bool = False
        after: Optional[str] = None
    
    class Store:
        pass

load_dotenv()
logger = logging.getLogger(__name__)


class NeonStore(Store):
    """
    PostgreSQL-backed store for ChatKit that persists chat history per user.
    Uses existing chat_threads and chat_messages tables.
    """
    
    def __init__(self):
        self.database_url = os.getenv('DATABASE_URL')
        if not self.database_url:
            raise ValueError("DATABASE_URL environment variable is not set")
        
        # Connection (will be recreated as needed for serverless DB compatibility)
        self._connection = None
    
    def _create_connection(self):
        """Create a new database connection"""
        return psycopg2.connect(
            dsn=self.database_url,
            sslmode='require',
            cursor_factory=RealDictCursor,
            connect_timeout=10
        )
    
    def _is_connection_alive(self, conn):
        """Check if connection is still alive by running a simple query"""
        if conn is None or conn.closed:
            return False
        try:
            with conn.cursor() as cursor:
                cursor.execute("SELECT 1")
            return True
        except Exception:
            return False
    
    def get_connection(self):
        """Get a valid database connection, reconnecting if necessary"""
        try:
            # Check if connection exists and is alive
            if not self._is_connection_alive(self._connection):
                # Close old connection if it exists
                if self._connection is not None:
                    try:
                        self._connection.close()
                    except Exception:
                        pass  # Ignore errors closing stale connection
                
                # Create new connection
                self._connection = self._create_connection()
                logger.debug("Created new database connection")
            
            return self._connection
        except Exception as e:
            logger.error(f"Failed to connect to database: {e}")
            raise
    
    def _get_user_id_from_context(self, context: dict) -> Optional[str]:
        """Extract user ID from context passed by the API endpoint"""
        if not context:
            return None
        user = context.get('user', {})
        return user.get('id')
    
    def get_user_profile(self, user_id: str) -> dict:
        """Fetch user profile details for personalization"""
        if not user_id:
            return {}
            
        conn = self.get_connection()
        try:
            with conn.cursor() as cursor:
                # Note: Table name is "user" (singular, quoted because it's a reserved word)
                cursor.execute("""
                    SELECT "educationLevel", "programmingExperience", "roboticsBackground"
                    FROM "user"
                    WHERE id = %s
                """, (user_id,))
                
                row = cursor.fetchone()
                if row:
                    return {
                        "education_level": row.get("educationLevel"),
                        "programming_experience": row.get("programmingExperience"),
                        "robotics_background": row.get("roboticsBackground")
                    }
                return {}
        except Exception as e:
            logger.error(f"Error fetching user profile for {user_id}: {e}")
            return {}
    
    def _ensure_timezone(self, dt: datetime) -> datetime:
        """Ensure datetime is timezone-aware"""
        if dt is None:
            return datetime.now(timezone.utc)
        if dt.tzinfo is None:
            return dt.replace(tzinfo=timezone.utc)
        return dt
    
    # ==================== ID Generation ====================
    
    def generate_thread_id(self, context: dict) -> str:
        """Generate unique thread ID"""
        return str(uuid.uuid4())
    
    def generate_item_id(self, item_type: str, thread: ThreadMetadata, context: dict) -> str:
        """Generate unique item ID with type prefix"""
        return f"{item_type}_{uuid.uuid4().hex[:12]}"
    
    # ==================== Thread Operations ====================
    
    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        """Load existing thread or create new one"""
        user_id = self._get_user_id_from_context(context)
        conn = self.get_connection()
        
        try:
            with conn.cursor() as cursor:
                # Try to load existing thread
                cursor.execute("""
                    SELECT id, user_id, title, created_at, updated_at
                    FROM chat_threads
                    WHERE id::text = %s
                """, (thread_id,))
                
                row = cursor.fetchone()
                
                if row:
                    # Check ownership if user_id provided
                    if user_id and row['user_id'] != user_id:
                        logger.warning(f"User {user_id} tried to access thread owned by {row['user_id']}")
                        # Return empty thread for unauthorized access
                        return ThreadMetadata(
                            id=thread_id,
                            created_at=datetime.now(timezone.utc),
                            metadata={}
                        )
                    
                    return ThreadMetadata(
                        id=str(row['id']),
                        created_at=self._ensure_timezone(row['created_at']),
                        metadata={'title': row['title'], 'user_id': row['user_id']}
                    )
                
                # Create new thread if not found
                if user_id:
                    cursor.execute("""
                        INSERT INTO chat_threads (id, user_id, title, created_at, updated_at)
                        VALUES (%s::uuid, %s, %s, %s, %s)
                        RETURNING id, user_id, title, created_at
                    """, (thread_id, user_id, 'New Conversation', datetime.now(timezone.utc), datetime.now(timezone.utc)))
                    
                    conn.commit()
                    new_row = cursor.fetchone()
                    
                    if new_row:
                        return ThreadMetadata(
                            id=str(new_row['id']),
                            created_at=self._ensure_timezone(new_row['created_at']),
                            metadata={'title': new_row['title'], 'user_id': new_row['user_id']}
                        )
                
                # Return in-memory thread for anonymous users
                return ThreadMetadata(
                    id=thread_id,
                    created_at=datetime.now(timezone.utc),
                    metadata={}
                )
                
        except Exception as e:
            logger.error(f"Error loading thread {thread_id}: {e}")
            conn.rollback()
            # Return in-memory thread on error
            return ThreadMetadata(
                id=thread_id,
                created_at=datetime.now(timezone.utc),
                metadata={}
            )
    
    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        """Save thread metadata"""
        user_id = self._get_user_id_from_context(context)
        if not user_id:
            return  # Don't persist anonymous users' threads
        
        conn = self.get_connection()
        
        try:
            with conn.cursor() as cursor:
                title = thread.metadata.get('title', 'New Conversation') if thread.metadata else 'New Conversation'
                
                cursor.execute("""
                    INSERT INTO chat_threads (id, user_id, title, created_at, updated_at)
                    VALUES (%s::uuid, %s, %s, %s, %s)
                    ON CONFLICT (id) DO UPDATE SET
                        title = EXCLUDED.title,
                        updated_at = CURRENT_TIMESTAMP
                """, (thread.id, user_id, title, self._ensure_timezone(thread.created_at), datetime.now(timezone.utc)))
                
                conn.commit()
                
        except Exception as e:
            logger.error(f"Error saving thread {thread.id}: {e}")
            conn.rollback()
    
    async def load_threads(self, limit: int, after: Optional[str], order: str, context: dict) -> Page[ThreadMetadata]:
        """List all threads for the current user with pagination"""
        user_id = self._get_user_id_from_context(context)
        
        if not user_id:
            # Return empty for anonymous users
            return Page(data=[], has_more=False)
        
        conn = self.get_connection()
        
        try:
            with conn.cursor() as cursor:
                order_dir = "DESC" if order == "desc" else "ASC"
                
                cursor.execute(f"""
                    SELECT id, user_id, title, created_at, updated_at
                    FROM chat_threads
                    WHERE user_id = %s
                    ORDER BY updated_at {order_dir}
                    LIMIT %s
                """, (user_id, limit + 1))
                
                rows = cursor.fetchall()
                
                has_more = len(rows) > limit
                threads = []
                
                for row in rows[:limit]:
                    threads.append(ThreadMetadata(
                        id=str(row['id']),
                        created_at=self._ensure_timezone(row['created_at']),
                        metadata={'title': row['title'], 'user_id': row['user_id']}
                    ))
                
                return Page(data=threads, has_more=has_more)
                
        except Exception as e:
            logger.error(f"Error loading threads for user {user_id}: {e}")
            return Page(data=[], has_more=False)
    
    async def delete_thread(self, thread_id: str, context: dict) -> None:
        """Delete thread and all its messages (cascade)"""
        user_id = self._get_user_id_from_context(context)
        if not user_id:
            return
        
        conn = self.get_connection()
        
        try:
            with conn.cursor() as cursor:
                # Only delete if owned by user
                cursor.execute("""
                    DELETE FROM chat_threads
                    WHERE id::text = %s AND user_id = %s
                """, (thread_id, user_id))
                
                conn.commit()
                
        except Exception as e:
            logger.error(f"Error deleting thread {thread_id}: {e}")
            conn.rollback()
    
    # ==================== Item Operations ====================
    
    async def load_thread_items(
        self,
        thread_id: str,
        after: Optional[str],
        limit: int,
        order: str,
        context: dict,
    ) -> Page[ThreadItem]:
        """Load thread items (messages) with pagination"""
        conn = self.get_connection()
        
        try:
            with conn.cursor() as cursor:
                order_dir = "DESC" if order == "desc" else "ASC"
                
                cursor.execute(f"""
                    SELECT id, thread_id, role, content, sources, selected_text_used, created_at
                    FROM chat_messages
                    WHERE thread_id::text = %s
                    ORDER BY created_at {order_dir}
                    LIMIT %s
                """, (thread_id, limit + 1))
                
                rows = cursor.fetchall()
                
                has_more = len(rows) > limit
                items = []
                
                for row in rows[:limit]:
                    item = self._row_to_thread_item(row, thread_id)
                    if item:
                        items.append(item)
                
                return Page(data=items, has_more=has_more)
                
        except Exception as e:
            logger.error(f"Error loading items for thread {thread_id}: {e}")
            return Page(data=[], has_more=False)
    
    def _row_to_thread_item(self, row: dict, thread_id: str) -> Optional[ThreadItem]:
        """Convert database row to ChatKit ThreadItem"""
        try:
            role = row['role'].lower()
            created_at = self._ensure_timezone(row['created_at'])
            
            # Parse sources if present
            sources = None
            if row.get('sources'):
                if isinstance(row['sources'], str):
                    sources = json.loads(row['sources'])
                else:
                    sources = row['sources']
            
            if CHATKIT_AVAILABLE:
                if role == 'assistant':
                    # Assistant messages use 'output_text' type
                    content = [{"type": "output_text", "text": row['content']}]
                    return AssistantMessageItem(
                        id=str(row['id']),
                        thread_id=thread_id,
                        created_at=created_at,
                        content=content
                    )
                elif role == 'user':
                    # User messages use 'input_text' type
                    content = [{"type": "input_text", "text": row['content']}]
                    return UserMessageItem(
                        id=str(row['id']),
                        thread_id=thread_id,
                        created_at=created_at,
                        content=content,
                        inference_options={}  # Required field
                    )
            
            return None
            
        except Exception as e:
            logger.error(f"Error converting row to ThreadItem: {e}")
            return None
    
    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Add or update item in thread"""
        user_id = self._get_user_id_from_context(context)
        if not user_id:
            return  # Don't persist for anonymous users
        
        conn = self.get_connection()
        
        try:
            # Serialize item content
            if hasattr(item, 'model_dump'):
                item_data = item.model_dump()
            elif hasattr(item, '__dict__'):
                item_data = item.__dict__
            else:
                item_data = {}
            
            # Determine role from item type
            item_type = type(item).__name__.lower()
            if 'assistant' in item_type:
                role = 'assistant'
            elif 'user' in item_type:
                role = 'user'
            else:
                role = 'user'
            
            # Extract text content
            content_text = ""
            if hasattr(item, 'content') and item.content:
                for c in item.content:
                    if hasattr(c, 'text'):
                        content_text = c.text
                        break
                    elif isinstance(c, dict) and 'text' in c:
                        content_text = c['text']
                        break
            
            # Skip empty content
            if not content_text:
                logger.debug(f"Skipping item with empty content for thread {thread_id}")
                return
            
            # Get created_at
            created_at = datetime.now(timezone.utc)
            if hasattr(item, 'created_at') and item.created_at:
                created_at = self._ensure_timezone(item.created_at)
            
            # Generate a proper UUID for database storage
            # ChatKit generates IDs like "message_66365fc59b37" which aren't valid UUIDs
            db_message_id = str(uuid.uuid4())
            
            with conn.cursor() as cursor:
                cursor.execute("""
                    INSERT INTO chat_messages (id, thread_id, role, content, sources, created_at)
                    VALUES (%s, %s, %s, %s, %s, %s)
                """, (db_message_id, thread_id, role, content_text, json.dumps(item_data.get('sources')), created_at))
                
                # Also update thread's updated_at
                cursor.execute("""
                    UPDATE chat_threads SET updated_at = CURRENT_TIMESTAMP
                    WHERE id::text = %s
                """, (thread_id,))
                
                conn.commit()
                logger.debug(f"Saved message {db_message_id} to thread {thread_id}")
                
        except Exception as e:
            logger.error(f"Error adding item to thread {thread_id}: {e}")
            conn.rollback()
    
    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Alias for add_thread_item"""
        await self.add_thread_item(thread_id, item, context)
    
    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        """Load single item by ID"""
        conn = self.get_connection()
        
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    SELECT id, thread_id, role, content, sources, selected_text_used, created_at
                    FROM chat_messages
                    WHERE id::text = %s AND thread_id::text = %s
                """, (item_id, thread_id))
                
                row = cursor.fetchone()
                
                if row:
                    item = self._row_to_thread_item(row, thread_id)
                    if item:
                        return item
                
                raise ValueError(f"Item {item_id} not found")
                
        except Exception as e:
            logger.error(f"Error loading item {item_id}: {e}")
            raise
    
    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        """Delete single item"""
        user_id = self._get_user_id_from_context(context)
        if not user_id:
            return
        
        conn = self.get_connection()
        
        try:
            with conn.cursor() as cursor:
                cursor.execute("""
                    DELETE FROM chat_messages
                    WHERE id::text = %s AND thread_id::text = %s
                """, (item_id, thread_id))
                
                conn.commit()
                
        except Exception as e:
            logger.error(f"Error deleting item {item_id}: {e}")
            conn.rollback()
    
    # ==================== Attachment Operations ====================
    # Simple in-memory implementation since we don't have an attachments table
    
    _attachments: dict = {}
    
    async def save_attachment(self, attachment: Any, context: dict) -> None:
        """Save attachment (in-memory for now)"""
        if hasattr(attachment, 'id'):
            NeonStore._attachments[attachment.id] = attachment
    
    async def load_attachment(self, attachment_id: str, context: dict) -> Any:
        """Load attachment by ID"""
        if attachment_id not in NeonStore._attachments:
            raise ValueError(f"Attachment {attachment_id} not found")
        return NeonStore._attachments[attachment_id]
    
    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        """Delete attachment"""
        NeonStore._attachments.pop(attachment_id, None)
