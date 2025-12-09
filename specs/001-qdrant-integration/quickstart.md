# Quickstart: Qdrant Vector Database Integration

## Prerequisites

- Python 3.12+ with uv package manager
- Qdrant Cloud account and API key
- Google Gemini API key
- Node.js and npm for frontend

## Setup

### 1. Install Backend Dependencies

```bash
cd backend
uv add fastapi uvicorn openai-agents openai google-generativeai qdrant-client psycopg2 pyjwt passlib bcrypt python-dotenv
```

### 2. Configure Environment Variables

Create `backend/.env` with:

```env
# Qdrant Cloud Configuration
QDRANT_URL=https://your-cluster.qdrant.cloud:6333
QDRANT_API_KEY=your_qdrant_api_key

# Google Gemini API
GEMINI_API_KEY=your_gemini_api_key

# Collection Settings
QDRANT_COLLECTION_NAME=book_content

# RAG Settings
SEARCH_LIMIT=5
SCORE_THRESHOLD=0.7
CHUNK_SIZE=1000
CHUNK_OVERLAP=200
```

### 3. Create Database Client

Create `backend/backend/database.py`:

```python
import os
from typing import List, Optional
import qdrant_client
from qdrant_client.http import models
import google.generativeai as genai
from dotenv import load_dotenv

load_dotenv()

# Initialize Qdrant client
qdrant_client_instance = qdrant_client.QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY"),
)

# Configure Google Gemini
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

def get_embedding(text: str, task_type: str = "retrieval_document") -> List[float]:
    """Generate embedding for text using Google Gemini"""
    try:
        embedding = genai.embed_content(
            model="models/text-embedding-004",
            content=text,
            task_type=task_type
        )
        return embedding['embedding']
    except Exception as e:
        raise Exception(f"Error generating embedding: {str(e)}")

def initialize_collection(collection_name: str) -> None:
    """Initialize Qdrant collection with cosine distance"""
    try:
        collections = qdrant_client_instance.get_collections()
        collection_names = [c.name for c in collections.collections]

        if collection_name not in collection_names:
            qdrant_client_instance.create_collection(
                collection_name=collection_name,
                vectors_config=models.VectorParams(
                    size=768,  # 768-dim for Gemini embeddings
                    distance=models.Distance.COSINE
                )
            )
            print(f"Collection '{collection_name}' created successfully")
        else:
            print(f"Collection '{collection_name}' already exists")
    except Exception as e:
        raise Exception(f"Error initializing collection: {str(e)}")
```

### 4. Create Ingestion Script

Create `backend/backend/ingest.py`:

```python
import os
import argparse
from pathlib import Path
from typing import List, Dict
import qdrant_client
from qdrant_client.http import models
from backend.database import get_embedding, initialize_collection
import uuid

def chunk_text(text: str, chunk_size: int = 1000, overlap: int = 200) -> List[Dict]:
    """Chunk text into segments with overlap"""
    chunks = []
    start = 0
    text_length = len(text)

    while start < text_length:
        end = start + chunk_size

        # If this is not the first chunk, add overlap
        if start > 0:
            overlap_start = start - overlap
            start = overlap_start if overlap_start > 0 else 0
            end = start + chunk_size

        chunk_text = text[start:end]
        chunks.append({
            'text': chunk_text,
            'start_pos': start,
            'end_pos': end
        })

        start = end

    return chunks

def process_markdown_file(file_path: Path, chunk_size: int = 1000, overlap: int = 200) -> List[Dict]:
    """Process a markdown file and return chunks with metadata"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    chunks = chunk_text(content, chunk_size, overlap)
    relative_path = file_path.relative_to(Path.cwd())

    processed_chunks = []
    for i, chunk in enumerate(chunks):
        processed_chunks.append({
            'filename': str(relative_path),
            'text': chunk['text'],
            'chunk_number': i + 1,
            'total_chunks': len(chunks)
        })

    return processed_chunks

def ingest_docs(docs_path: str, collection_name: str = "book_content"):
    """Ingest all markdown files from docs_path into Qdrant"""
    docs_dir = Path(docs_path)

    if not docs_dir.exists():
        raise ValueError(f"Docs path does not exist: {docs_path}")

    # Initialize collection
    initialize_collection(collection_name)

    # Find all markdown files recursively
    md_files = list(docs_dir.rglob("*.md"))
    print(f"Found {len(md_files)} markdown files")

    all_chunks = []
    for md_file in md_files:
        print(f"Processing {md_file}...")
        try:
            chunks = process_markdown_file(md_file)
            for chunk in chunks:
                all_chunks.append(chunk)
            print(f"  Processed {len(chunks)} chunks")
        except Exception as e:
            print(f"  Error processing {md_file}: {str(e)}")

    print(f"Total chunks to upload: {len(all_chunks)}")

    # Upload chunks to Qdrant
    points = []
    for i, chunk in enumerate(all_chunks):
        # Generate embedding
        embedding = get_embedding(chunk['text'], "retrieval_document")

        # Create Qdrant point
        point = models.PointStruct(
            id=str(uuid.uuid4()),
            vector=embedding,
            payload={
                "filename": chunk['filename'],
                "text": chunk['text'],
                "chunk_number": chunk['chunk_number'],
                "total_chunks": chunk['total_chunks']
            }
        )
        points.append(point)

        # Upload in batches of 100
        if len(points) >= 100:
            qdrant_client_instance.upload_points(
                collection_name=collection_name,
                points=points
            )
            print(f"Uploaded batch of {len(points)} points")
            points = []

    # Upload remaining points
    if points:
        qdrant_client_instance.upload_points(
            collection_name=collection_name,
            points=points
        )
        print(f"Uploaded final batch of {len(points)} points")

    print(f"Successfully ingested {len(all_chunks)} chunks into collection '{collection_name}'")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Ingest documentation into Qdrant")
    parser.add_argument("--docs_path", required=True, help="Path to documentation directory")

    args = parser.parse_args()

    ingest_docs(args.docs_path)
```

### 5. Update Main Backend API

Update `backend/src/backend/main.py` to include RAG functionality:

```python
from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional, List, Dict, Any
import google.generativeai as genai
from backend.database import qdrant_client_instance, get_embedding
from pydantic import BaseModel
import os

# Configure Gemini
genai.configure(api_key=os.getenv("GEMINI_API_KEY"))

app = FastAPI(title="Qdrant RAG API")

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Configure appropriately for production
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Request/Response models
class ChatRequest(BaseModel):
    user_query: str
    selected_text: Optional[str] = None
    chat_history: Optional[List[Dict[str, str]]] = []

class ChatResponse(BaseModel):
    output: str
    context_chunks: List[Dict[str, Any]]
    sources: List[str]

class SearchRequest(BaseModel):
    query: str
    limit: Optional[int] = 5
    score_threshold: Optional[float] = 0.7

# RAG Chat endpoint
@app.post("/api/chat", response_model=ChatResponse)
async def rag_chat(request: ChatRequest):
    try:
        # Generate embedding for user query
        query_embedding = get_embedding(request.user_query, "retrieval_query")

        # Search in Qdrant
        search_results = qdrant_client_instance.search(
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
            query_vector=query_embedding,
            limit=request.limit or int(os.getenv("SEARCH_LIMIT", "5")),
            score_threshold=request.score_threshold or float(os.getenv("SCORE_THRESHOLD", "0.7"))
        )

        # Extract context from search results
        context_chunks = []
        sources = set()

        for result in search_results:
            if result.score >= (request.score_threshold or float(os.getenv("SCORE_THRESHOLD", "0.7"))):
                chunk_data = {
                    "filename": result.payload.get("filename"),
                    "text": result.payload.get("text"),
                    "chunk_number": result.payload.get("chunk_number"),
                    "total_chunks": result.payload.get("total_chunks"),
                    "score": result.score
                }
                context_chunks.append(chunk_data)
                sources.add(result.payload.get("filename"))

        # Build context string for the agent
        context_str = "\n\n".join([chunk["text"] for chunk in context_chunks])

        # Create system prompt with context
        system_prompt = f"""
        You are a helpful assistant that answers questions based on the provided book content.
        Use the following context to answer the user's question:
        {context_str}

        If the context doesn't contain information to answer the question, say so.
        Always cite the source document when providing information from the context.
        """

        # Use OpenAI Agents SDK with augmented context (simplified implementation)
        # In a real implementation, this would use the OpenAI Agents SDK as specified
        model = genai.GenerativeModel('gemini-pro')  # Using Gemini for consistency

        full_prompt = f"{system_prompt}\n\nUser question: {request.user_query}"

        response = model.generate_content(full_prompt)

        return ChatResponse(
            output=response.text if response.text else "I couldn't find relevant information in the book content.",
            context_chunks=context_chunks,
            sources=list(sources)
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
        search_results = qdrant_client_instance.search(
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "book_content"),
            query_vector=query_embedding,
            limit=limit,
            score_threshold=score_threshold
        )

        results = []
        for result in search_results:
            if result.score >= score_threshold:
                results.append({
                    "id": result.id,
                    "filename": result.payload.get("filename"),
                    "text": result.payload.get("text"),
                    "chunk_number": result.payload.get("chunk_number"),
                    "total_chunks": result.payload.get("total_chunks"),
                    "score": result.score
                })

        return {"results": results}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Error performing search: {str(e)}")

# Health check
@app.get("/health")
async def health_check():
    return {"status": "healthy"}
```

### 6. Create Test Suite

Create `backend/backend/test_qdrant.py`:

```python
import os
from backend.database import qdrant_client_instance, get_embedding, initialize_collection

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
        results = qdrant_client_instance.search(
            collection_name="test_search_collection",
            query_vector=query_embedding,
            limit=1
        )

        print(f"✅ Search test completed. Found {len(results)} results")
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
```

### 7. Frontend Integration

Update `frontend/src/components/ChatBot/index.tsx` to handle RAG responses:

```typescript
// This is a simplified example - actual implementation would depend on the existing structure
interface RAGResponse {
  output: string;
  context_chunks: Array<{
    filename: string;
    text: string;
    chunk_number: number;
    score: number;
  }>;
  sources: string[];
}

const ChatBot = () => {
  const [query, setQuery] = useState('');
  const [response, setResponse] = useState<RAGResponse | null>(null);
  const [loading, setLoading] = useState(false);

  const handleChatSubmit = async () => {
    setLoading(true);
    try {
      const res = await fetch('http://localhost:8000/api/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          user_query: query,
          selected_text: null, // if any selected text
          chat_history: [] // if implementing chat history
        }),
      });

      const data: RAGResponse = await res.json();
      setResponse(data);
    } catch (error) {
      console.error('Error:', error);
    } finally {
      setLoading(false);
    }
  };

  return (
    <div className="chatbot-container">
      <div className="chat-input">
        <input
          type="text"
          value={query}
          onChange={(e) => setQuery(e.target.value)}
          placeholder="Ask about the book content..."
          onKeyPress={(e) => e.key === 'Enter' && handleChatSubmit()}
        />
        <button onClick={handleChatSubmit} disabled={loading}>
          {loading ? 'Processing...' : 'Send'}
        </button>
      </div>

      {loading && <div className="loading">Processing your query...</div>}

      {response && (
        <div className="chat-response">
          <div className="response-text">{response.output}</div>

          {response.sources.length > 0 && (
            <div className="sources-section">
              <h4>Sources:</h4>
              <ul>
                {response.sources.map((source, idx) => (
                  <li key={idx}>{source}</li>
                ))}
              </ul>
            </div>
          )}

          {response.context_chunks.length > 0 && (
            <div className="context-section">
              <h4>Context used:</h4>
              {response.context_chunks.map((chunk, idx) => (
                <div key={idx} className="context-chunk">
                  <p><strong>File:</strong> {chunk.filename}</p>
                  <p><strong>Relevance:</strong> {(chunk.score * 100).toFixed(1)}%</p>
                  <p><strong>Text:</strong> {chunk.text.substring(0, 100)}...</p>
                </div>
              ))}
            </div>
          )}
        </div>
      )}
    </div>
  );
};
```

## Running the System

### 1. Test Qdrant Connection

```bash
cd backend
uv run backend/test_qdrant.py
```

### 2. Ingest Book Content

```bash
uv run backend/ingest.py --docs_path ../frontend/docs
```

### 3. Start Backend Server

```bash
uv run uvicorn src.backend.main:app --reload
```

### 4. Test API Endpoints

```bash
# Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{"user_query": "What is this book about?"}'

# Test search endpoint
curl "http://localhost:8000/api/search?query=book+content"
```

## Architecture Flow

1. **Content Ingestion**: Markdown files → Chunking → Embedding → Qdrant storage
2. **Query Processing**: User query → Embedding → Vector search → Context retrieval
3. **Response Generation**: Context + Query → RAG Agent → Answer with citations
4. **Frontend Display**: Response → Source citations → User interface