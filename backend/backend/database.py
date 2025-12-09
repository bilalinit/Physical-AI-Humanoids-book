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