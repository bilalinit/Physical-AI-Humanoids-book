from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from typing import Optional, List, Dict, Any
import os
from pydantic import BaseModel
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# Import OpenAI Agents SDK components
from agents import Agent, Runner, OpenAIChatCompletionsModel, RunConfig, AsyncOpenAI

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

# Configure the run settings
config = RunConfig(
    model=model,
    model_provider=client,
)

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

# Import Qdrant functionality from database module
import sys
import os
from pathlib import Path

# Add the parent backend directory to the path to import backend modules
sys.path.append(str(Path(__file__).parent.parent.parent))

from backend.database import qdrant_client_instance, get_embedding

# RAG Chat endpoint
@app.post("/api/chat", response_model=ChatResponse)
async def rag_chat(request: ChatRequest):
    try:
        # Simple content validation (without complex guardrail that's causing issues)
        user_query = request.user_query
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

        # Create system prompt with context
        system_prompt = f"""
        You are a helpful assistant that answers questions based on the provided book content.
        Use the following context to answer the user's question:
        {context_str}

        If the context doesn't contain information to answer the question, say so.
        Always cite the source document when providing information from the context.
        """

        # Create the RAG agent with the context
        rag_agent = Agent(
            name="RAGBot",
            instructions=system_prompt,
            model=model
        )

        # Run the agent with the user query
        response = await Runner.run(rag_agent, user_query, run_config=config)

        return ChatResponse(
            output=response.final_output if response.final_output else "I couldn't find relevant information in the book content.",
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