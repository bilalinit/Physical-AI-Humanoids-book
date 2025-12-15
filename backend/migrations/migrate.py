import os
import sys
import psycopg2
from psycopg2.extras import RealDictCursor
from dotenv import load_dotenv
import glob

# Load environment variables
# Attempt to load from parent directory if not found in CWD
if os.path.exists('.env'):
    load_dotenv('.env')
elif os.path.exists('../.env'):
    load_dotenv('../.env')

def get_db_connection():
    """Create a database connection using environment variables"""
    dsn = os.getenv('DATABASE_URL')
    if not dsn:
        print("Error: DATABASE_URL environment variable not found.")
        sys.exit(1)
        
    conn = psycopg2.connect(
        dsn=dsn,
        sslmode='require'
    )
    return conn

def run_migrations():
    """Run all migration files in order"""
    try:
        conn = get_db_connection()
        cursor = conn.cursor(cursor_factory=RealDictCursor)

        # Create migrations table if it doesn't exist
        cursor.execute("""
            CREATE TABLE IF NOT EXISTS migrations (
                id SERIAL PRIMARY KEY,
                migration_name VARCHAR(255) UNIQUE NOT NULL,
                executed_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
            )
        """)
        conn.commit()

        # Get already executed migrations
        cursor.execute("SELECT migration_name FROM migrations ORDER BY migration_name")
        executed_migrations = [row['migration_name'] for row in cursor.fetchall()]

        # Get all migration files
        # Look for sql files in the same directory as this script
        script_dir = os.path.dirname(os.path.abspath(__file__))
        migration_files = glob.glob(os.path.join(script_dir, "*.sql"))
        migration_files.sort()  # Sort to ensure order

        print(f"Found {len(migration_files)} migration files in {script_dir}")

        for migration_file in migration_files:
            migration_name = os.path.basename(migration_file)

            # Skip if already executed
            if migration_name in executed_migrations:
                print(f"Skipping {migration_name} (already executed)")
                continue

            print(f"Executing migration: {migration_name}")

            # Read and execute migration
            with open(migration_file, 'r') as f:
                sql = f.read()

            cursor.execute(sql)

            # Record migration execution
            cursor.execute(
                "INSERT INTO migrations (migration_name) VALUES (%s)",
                (migration_name,)
            )

            conn.commit()
            print(f"Migration {migration_name} completed successfully")

        cursor.close()
        conn.close()

        print("All migrations completed!")

    except Exception as e:
        print(f"Error running migrations: {str(e)}")
        sys.exit(1)

if __name__ == "__main__":
    run_migrations()
