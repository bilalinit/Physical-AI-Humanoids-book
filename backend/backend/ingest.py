import os
import argparse
from pathlib import Path
from typing import List, Dict
import qdrant_client
from qdrant_client.http import models
import sys

# Add the parent directory to the Python path so we can import from database
sys.path.append(str(Path(__file__).parent))

from database import get_embedding, initialize_collection, qdrant_client_instance
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

def process_markdown_file(file_path: Path, docs_dir: Path, chunk_size: int = 1000, overlap: int = 200) -> List[Dict]:
    """Process a markdown file and return chunks with metadata"""
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    chunks = chunk_text(content, chunk_size, overlap)
    # Calculate relative path from the docs directory, not current working directory
    relative_path = file_path.relative_to(docs_dir)

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
            chunks = process_markdown_file(md_file, docs_dir)
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