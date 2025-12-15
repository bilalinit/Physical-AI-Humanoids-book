"""
Database migration framework for schema evolution in Neon PostgreSQL
"""
import os
from sqlalchemy import create_engine, text
from dotenv import load_dotenv
from .database import Base, engine

load_dotenv()

def run_migrations():
    """
    Run database migrations to create/update tables based on SQLAlchemy models
    """
    try:
        # Create all tables defined in the models
        Base.metadata.create_all(bind=engine)
        print("Database tables created/updated successfully")

        # Create indexes for performance
        create_indexes()

        # Create constraints for data integrity
        create_constraints()

    except Exception as e:
        print(f"Error running migrations: {str(e)}")
        raise

def create_indexes():
    """
    Create database indexes for performance optimization
    """
    with engine.connect() as conn:
        # User table indexes
        try:
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_user_email ON \"user\"(\"email\");"))
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_user_created_at ON \"user\"(\"createdAt\" DESC);"))
        except Exception as e:
            # Index might already exist, which is fine
            print(f"User indexes: {str(e)}")

        # Session table indexes
        try:
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_session_user_id ON \"session\"(\"userId\");"))
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_session_expires_at ON \"session\"(\"expiresAt\");"))
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_session_token ON \"session\"(\"token\");"))
        except Exception as e:
            # Index might already exist, which is fine
            print(f"Session indexes: {str(e)}")

        # Chat history table indexes
        try:
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_chat_history_user_id ON \"chat_history\"(\"userId\");"))
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_chat_history_created_at ON \"chat_history\"(\"createdAt\" DESC);"))
            conn.execute(text("CREATE INDEX IF NOT EXISTS idx_chat_history_user_created ON \"chat_history\"(\"userId\", \"createdAt\" DESC);"))
        except Exception as e:
            # Index might already exist, which is fine
            print(f"Chat history indexes: {str(e)}")

        conn.commit()

def create_constraints():
    """
    Create database constraints for data integrity
    """
    with engine.connect() as conn:
        # User constraints
        try:
            # Email format check constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_user_email'
                    ) THEN
                        ALTER TABLE "user" ADD CONSTRAINT chk_user_email
                        CHECK (email ~* '^[A-Za-z0-9._%+-]+@[A-Za-z0-9.-]+\.[A-Za-z]{2,}$');
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Email constraint: {str(e)}")

        try:
            # Name length check constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_user_name_length'
                    ) THEN
                        ALTER TABLE "user" ADD CONSTRAINT chk_user_name_length
                        CHECK (LENGTH("name") BETWEEN 2 AND 100);
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Name length constraint: {str(e)}")

        try:
            # Education level enum constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_education_level'
                    ) THEN
                        ALTER TABLE "user" ADD CONSTRAINT chk_education_level
                        CHECK ("educationLevel" IN ('High School', 'Undergraduate', 'Graduate', 'Professional') OR "educationLevel" IS NULL);
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Education level constraint: {str(e)}")

        try:
            # Programming experience enum constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_programming_experience'
                    ) THEN
                        ALTER TABLE "user" ADD CONSTRAINT chk_programming_experience
                        CHECK ("programmingExperience" IN ('No Experience', 'Beginner', 'Intermediate', 'Advanced') OR "programmingExperience" IS NULL);
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Programming experience constraint: {str(e)}")

        try:
            # Robotics background enum constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_robotics_background'
                    ) THEN
                        ALTER TABLE "user" ADD CONSTRAINT chk_robotics_background
                        CHECK ("roboticsBackground" IN ('No Experience', 'Hobbyist', 'Academic', 'Professional') OR "roboticsBackground" IS NULL);
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Robotics background constraint: {str(e)}")

        try:
            # Chat history message length constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_message_length'
                    ) THEN
                        ALTER TABLE "chat_history" ADD CONSTRAINT chk_message_length
                        CHECK (LENGTH("message") BETWEEN 1 AND 5000);
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Message length constraint: {str(e)}")

        try:
            # Chat history response length constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_response_length'
                    ) THEN
                        ALTER TABLE "chat_history" ADD CONSTRAINT chk_response_length
                        CHECK (LENGTH("response") BETWEEN 1 AND 10000);
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Response length constraint: {str(e)}")

        try:
            # Chat history selected text length constraint
            conn.execute(text("""
                DO $$
                BEGIN
                    IF NOT EXISTS (
                        SELECT 1 FROM information_schema.check_constraints
                        WHERE constraint_name = 'chk_selected_text_length'
                    ) THEN
                        ALTER TABLE "chat_history" ADD CONSTRAINT chk_selected_text_length
                        CHECK (LENGTH("selectedText") BETWEEN 0 AND 2000 OR "selectedText" IS NULL);
                    END IF;
                END $$;
            """))
        except Exception as e:
            # Constraint might already exist, which is fine
            print(f"Selected text length constraint: {str(e)}")

        conn.commit()

def rollback_migration(migration_name):
    """
    Rollback a specific migration if needed
    """
    print(f"Rolling back migration: {migration_name}")
    # Implementation would depend on specific migration needs
    pass

if __name__ == "__main__":
    run_migrations()