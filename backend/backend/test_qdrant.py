import os
import sys
from pathlib import Path

# Load environment variables first
from dotenv import load_dotenv
load_dotenv()

# Add the parent directory to the Python path so we can import from backend.database
sys.path.append(str(Path(__file__).parent))

from database import qdrant_client_instance, get_embedding, initialize_collection

def test_connection():
    """Test Qdrant connectivity"""
    try:
        collections = qdrant_client_instance.get_collections()
        print("✅ Qdrant connection successful")
        print(f"Available collections: {[c.name for c in collections.collections]}")
        return True
    except Exception as e:
        print(f"❌ Qdrant connection failed: {str(e)}")
        return False

def test_embedding():
    """Test embedding generation"""
    try:
        text = "This is a test sentence for embedding."
        embedding = get_embedding(text)
        print(f"✅ Embedding generated successfully. Length: {len(embedding)}")
        return True
    except Exception as e:
        print(f"❌ Embedding generation failed: {str(e)}")
        return False

def test_upsert():
    """Test uploading a test point to Qdrant"""
    try:
        from qdrant_client.http import models
        import uuid

        # Initialize collection
        initialize_collection("test_collection")

        # Create a test embedding
        embedding = get_embedding("Test document for upsert")

        # Create and upload a test point
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "filename": "test.md",
                "text": "Test document for upsert",
                "chunk_number": 1,
                "total_chunks": 1
            }
        )

        qdrant_client_instance.upload_points(
            collection_name="test_collection",
            points=[point]
        )

        print("✅ Test point uploaded successfully")
        return True
    except Exception as e:
        print(f"❌ Test upsert failed: {str(e)}")
        return False

def test_search():
    """Test vector search functionality"""
    try:
        # Initialize collection
        initialize_collection("test_search_collection")

        # Create and upload a test point
        from qdrant_client.http import models
        import uuid

        test_embedding = get_embedding("Sample document for search testing")
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=test_embedding,
            payload={
                "filename": "search_test.md",
                "text": "Sample document for search testing",
                "chunk_number": 1,
                "total_chunks": 1
            }
        )

        qdrant_client_instance.upload_points(
            collection_name="test_search_collection",
            points=[point]
        )

        # Perform search
        query_embedding = get_embedding("search for sample document", "retrieval_query")
        search_response = qdrant_client_instance.query_points(
            collection_name="test_search_collection",
            query=query_embedding,
            limit=1
        )

        # Extract results from the response
        search_results = search_response.points if hasattr(search_response, 'points') else search_response
        results_count = len(search_results) if hasattr(search_results, '__len__') else len(list(search_results))

        print(f"✅ Search test completed. Found {results_count} results")
        return True
    except Exception as e:
        print(f"❌ Search test failed: {str(e)}")
        return False

if __name__ == "__main__":
    print("Running Qdrant tests...\n")

    tests = [
        ("Connection Test", test_connection),
        ("Embedding Test", test_embedding),
        ("Upsert Test", test_upsert),
        ("Search Test", test_search)
    ]

    results = []
    for test_name, test_func in tests:
        print(f"Running {test_name}...")
        success = test_func()
        results.append((test_name, success))
        print()

    print("Test Results:")
    for test_name, success in results:
        status = "✅ PASS" if success else "❌ FAIL"
        print(f"  {test_name}: {status}")

    all_passed = all(result[1] for result in results)
    print(f"\nOverall: {'✅ ALL TESTS PASSED' if all_passed else '❌ SOME TESTS FAILED'}")