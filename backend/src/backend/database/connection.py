"""
Async database connection management for chat history persistence
"""
import os
from contextlib import asynccontextmanager
from typing import AsyncGenerator

from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, async_sessionmaker
from sqlalchemy.orm import sessionmaker
from sqlalchemy import event
from dotenv import load_dotenv

load_dotenv()

# Database configuration for Neon PostgreSQL
DATABASE_URL = os.getenv("DATABASE_URL")

# Replace postgresql:// with postgresql+asyncpg:// for async support
if DATABASE_URL:
    if DATABASE_URL.startswith("postgresql://"):
        ASYNC_DATABASE_URL = DATABASE_URL.replace("postgresql://", "postgresql+asyncpg://", 1)
    elif DATABASE_URL.startswith("postgres://"):
        ASYNC_DATABASE_URL = DATABASE_URL.replace("postgres://", "postgresql+asyncpg://", 1)
    else:
        ASYNC_DATABASE_URL = DATABASE_URL
else:
    raise ValueError("DATABASE_URL environment variable is not set")

# Create async SQLAlchemy engine for Neon PostgreSQL
async_engine = create_async_engine(
    ASYNC_DATABASE_URL,
    pool_pre_ping=True,  # Verify connections before use
    pool_recycle=300,    # Recycle connections every 5 minutes
    echo=False,          # Set to True for SQL query logging
    # Additional async-specific settings
    pool_size=5,
    max_overflow=10,
    pool_timeout=30,
    poolclass=None
)

# Create async session factory
AsyncSessionLocal = async_sessionmaker(
    async_engine,
    class_=AsyncSession,
    expire_on_commit=False
)

@asynccontextmanager
async def get_async_db() -> AsyncGenerator[AsyncSession, None]:
    """
    Async context manager for database sessions
    """
    async with AsyncSessionLocal() as session:
        try:
            yield session
        finally:
            await session.close()

def get_sync_db():
    """
    Sync database session for compatibility with existing code
    """
    from backend.database import SessionLocal
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

# Add event listener to handle connection issues
@event.listens_for(async_engine.sync_engine, "connect")
def set_sqlite_pragma(dbapi_connection, connection_record):
    """
    Set SQLite pragmas for better performance (only relevant if using SQLite)
    """
    if 'sqlite' in ASYNC_DATABASE_URL:
        cursor = dbapi_connection.cursor()
        # Configure SQLite for better concurrency
        cursor.execute("PRAGMA foreign_keys=ON")
        cursor.execute("PRAGMA journal_mode=WAL")
        cursor.execute("PRAGMA synchronous=NORMAL")
        cursor.close()