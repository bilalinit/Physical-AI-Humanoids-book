import uuid
from datetime import datetime, timezone
from typing import Any
from dataclasses import dataclass, field

# Check if the chatkit modules are available, otherwise use a basic implementation
try:
    from chatkit.store import Store
    from chatkit.types import ThreadMetadata, ThreadItem, Page
    CHATKIT_AVAILABLE = True
except ImportError:
    # Fallback implementation if chatkit is not available
    from dataclasses import dataclass, field
    from typing import TypeVar, Generic, List, Optional, Dict, Any

    T = TypeVar('T')

    @dataclass
    class ThreadMetadata:
        id: str
        created_at: datetime
        metadata: Dict[str, Any] = field(default_factory=dict)

    @dataclass
    class ThreadItem:
        id: str
        type: str  # "message", "tool_call", etc.
        created_at: datetime
        content: Optional[str] = None

    @dataclass
    class Page(Generic[T]):
        data: List[T]
        has_more: bool = False
        after: Optional[str] = None

    class Store(Generic[T]):
        """Base store class for ChatKit"""
        def generate_thread_id(self, context: T) -> str:
            raise NotImplementedError

        def generate_item_id(self, item_type: str, thread, context: T) -> str:
            raise NotImplementedError

        def load_thread(self, thread_id: str, context: T):
            raise NotImplementedError

        def save_thread(self, thread, context: T) -> None:
            raise NotImplementedError

        def load_threads(self, limit: int, after: Optional[str], order: str, context: T):
            raise NotImplementedError

        def delete_thread(self, thread_id: str, context: T) -> None:
            raise NotImplementedError

        def load_thread_items(self, thread_id: str, after: Optional[str], limit: int, order: str, context: T):
            raise NotImplementedError

        def add_thread_item(self, thread_id: str, item, context: T) -> None:
            raise NotImplementedError

        def save_item(self, thread_id: str, item, context: T) -> None:
            raise NotImplementedError

        def load_item(self, thread_id: str, item_id: str, context: T):
            raise NotImplementedError

        def delete_thread_item(self, thread_id: str, item_id: str, context: T) -> None:
            raise NotImplementedError

        def save_attachment(self, attachment, context: T) -> None:
            raise NotImplementedError

        def load_attachment(self, attachment_id: str, context: T):
            raise NotImplementedError

        def delete_attachment(self, attachment_id: str, context: T) -> None:
            raise NotImplementedError

    CHATKIT_AVAILABLE = False


@dataclass
class ThreadState:
    """Internal state for a thread"""
    thread: ThreadMetadata
    items: list[ThreadItem] = field(default_factory=list)


class MemoryStore(Store[dict]):
    """Thread-safe in-memory store for ChatKit"""

    def __init__(self) -> None:
        self._threads: dict[str, ThreadState] = {}
        self._attachments: dict[str, Any] = {}

    # ==================== ID Generation ====================

    def generate_thread_id(self, context: dict) -> str:
        """Generate unique thread ID"""
        return f"thread_{uuid.uuid4().hex[:12]}"

    def generate_item_id(self, item_type: str, thread: ThreadMetadata, context: dict) -> str:
        """Generate unique item ID with type prefix"""
        return f"{item_type}_{uuid.uuid4().hex[:12]}"

    # ==================== Thread Operations ====================

    async def load_thread(self, thread_id: str, context: dict) -> ThreadMetadata:
        """Load existing thread or create new one"""
        state = self._threads.get(thread_id)
        if state:
            return state.thread.model_copy(deep=True)

        # Create new thread
        thread = ThreadMetadata(
            id=thread_id,
            created_at=datetime.now(timezone.utc),
            metadata={}
        )
        self._threads[thread_id] = ThreadState(
            thread=thread.model_copy(deep=True),
            items=[]
        )
        return thread

    async def save_thread(self, thread: ThreadMetadata, context: dict) -> None:
        """Save thread metadata"""
        state = self._threads.get(thread.id)
        if state:
            state.thread = thread.model_copy(deep=True)
        else:
            self._threads[thread.id] = ThreadState(
                thread=thread.model_copy(deep=True),
                items=[]
            )

    async def load_threads(self, limit: int, after: str | None, order: str, context: dict) -> Page[ThreadMetadata]:
        """List all threads with pagination"""
        threads = [s.thread.model_copy(deep=True) for s in self._threads.values()]
        # Sort by created_at
        threads.sort(
            key=lambda t: t.created_at,
            reverse=(order == "desc")
        )
        return Page(data=threads[:limit], has_more=len(threads) > limit)

    async def delete_thread(self, thread_id: str, context: dict) -> None:
        """Delete thread and all its items"""
        self._threads.pop(thread_id, None)

    # ==================== Item Operations ====================

    def _get_items(self, thread_id: str) -> list[ThreadItem]:
        """Helper to get items for a thread"""
        state = self._threads.get(thread_id)
        return state.items if state else []

    async def load_thread_items(
        self,
        thread_id: str,
        after: str | None,
        limit: int,
        order: str,
        context: dict,
    ) -> Page[ThreadItem]:
        """Load thread items with pagination"""
        items = [item.model_copy(deep=True) for item in self._get_items(thread_id)]

        # Sort by created_at
        items.sort(
            key=lambda i: getattr(i, "created_at", datetime.now(timezone.utc)),
            reverse=(order == "desc"),
        )

        # Handle 'after' cursor for pagination
        start = 0
        if after:
            index_map = {item.id: idx for idx, item in enumerate(items)}
            start = index_map.get(after, -1) + 1

        # Slice with limit + 1 to check has_more
        slice_items = items[start: start + limit + 1]
        has_more = len(slice_items) > limit
        result_items = slice_items[:limit]

        return Page(
            data=result_items,
            has_more=has_more,
            after=slice_items[-1].id if has_more and slice_items else None
        )

    async def add_thread_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Add or update item in thread"""
        state = self._threads.get(thread_id)
        if not state:
            await self.load_thread(thread_id, context)
            state = self._threads[thread_id]

        # Update if exists
        for i, existing in enumerate(state.items):
            if existing.id == item.id:
                state.items[i] = item.model_copy(deep=True)
                return

        # Add new
        state.items.append(item.model_copy(deep=True))

    async def save_item(self, thread_id: str, item: ThreadItem, context: dict) -> None:
        """Alias for add_thread_item"""
        await self.add_thread_item(thread_id, item, context)

    async def load_item(self, thread_id: str, item_id: str, context: dict) -> ThreadItem:
        """Load single item by ID"""
        for item in self._get_items(thread_id):
            if item.id == item_id:
                return item.model_copy(deep=True)
        raise ValueError(f"Item {item_id} not found")

    async def delete_thread_item(self, thread_id: str, item_id: str, context: dict) -> None:
        """Delete single item"""
        state = self._threads.get(thread_id)
        if state:
            state.items = [i for i in state.items if i.id != item_id]

    # ==================== Attachment Operations ====================
    # These are often forgotten but REQUIRED!

    async def save_attachment(self, attachment: Any, context: dict) -> None:
        """Save attachment"""
        self._attachments[attachment.id] = attachment

    async def load_attachment(self, attachment_id: str, context: dict) -> Any:
        """Load attachment by ID"""
        if attachment_id not in self._attachments:
            raise ValueError(f"Attachment {attachment_id} not found")
        return self._attachments[attachment_id]

    async def delete_attachment(self, attachment_id: str, context: dict) -> None:
        """Delete attachment"""
        self._attachments.pop(attachment_id, None)