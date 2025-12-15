#!/usr/bin/env python3
"""
Database migration script for Better Auth + Neon integration
Creates all necessary tables for the application
"""
import os
import sys
from pathlib import Path

# Add the project root to the path to import backend modules
project_root = Path(__file__).parent.parent.parent.parent  # /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/
backend_root = project_root / "backend"  # /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/backend/
backend_src = backend_root / "src" / "backend"  # /mnt/d/coding Q4/main-hackathon/save-7/hackathon-book/backend/src/backend/

# Add both paths to sys.path
sys.path.insert(0, str(backend_root))  # For backend.database import
sys.path.insert(0, str(backend_src))  # For src.backend.models import

from backend.database import create_tables, engine
from models.base import Base
from models.user import User
from models.session import Session
from models.chat_history import ChatHistory

def run_migrations():
    """Run database migrations to create all tables"""
    print("Starting database migrations...")

    try:
        # Create all tables
        create_tables()
        print("✓ All database tables created successfully!")

        # Verify tables were created
        print("\nVerifying table creation...")
        tables = Base.metadata.tables.keys()
        print(f"Created tables: {list(tables)}")

        # Test database connection
        print("\nTesting database connection...")
        from sqlalchemy import text
        with engine.connect() as conn:
            result = conn.execute(text("SELECT 1"))
            print("✓ Database connection successful!")

        print("\n✓ Database migration completed successfully!")
        return True

    except Exception as e:
        print(f"✗ Error during migration: {str(e)}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = run_migrations()
    sys.exit(0 if success else 1)