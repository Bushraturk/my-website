"""
Document Ingestion Script for RAG Chatbot
Indexes all markdown documents from docs/ folder into Qdrant vector store
"""

import os
import sys
import glob
import uuid
import re
from pathlib import Path
from typing import List, Dict
from dotenv import load_dotenv

# Load environment variables
load_dotenv()

# You need to set these environment variables:
# QDRANT_URL - Your Qdrant cloud URL
# QDRANT_API_KEY - Your Qdrant API key
# COHERE_API_KEY - Your Cohere API key

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
COHERE_API_KEY = os.getenv("COHERE_API_KEY")

# Validate environment variables
if not all([QDRANT_URL, QDRANT_API_KEY, COHERE_API_KEY]):
    print("ERROR: Missing required environment variables!")
    print(f"  QDRANT_URL: {'Set' if QDRANT_URL else 'MISSING'}")
    print(f"  QDRANT_API_KEY: {'Set' if QDRANT_API_KEY else 'MISSING'}")
    print(f"  COHERE_API_KEY: {'Set' if COHERE_API_KEY else 'MISSING'}")
    print("\nPlease create a .env file with these variables or set them in your environment.")
    sys.exit(1)

import cohere
from qdrant_client import QdrantClient
from qdrant_client.http import models

# Initialize clients
print("Initializing Cohere and Qdrant clients...")
co = cohere.Client(COHERE_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY, https=True)

COLLECTION_NAME = "book_content"
CHUNK_SIZE = 1000  # Characters per chunk
EMBEDDING_MODEL = "embed-english-v3.0"


def setup_collection():
    """Create collection if it doesn't exist"""
    try:
        qdrant.get_collection(COLLECTION_NAME)
        print(f"Collection '{COLLECTION_NAME}' already exists")
    except:
        print(f"Creating collection '{COLLECTION_NAME}'...")
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(
                size=1024,  # Cohere embed-english-v3.0 dimension
                distance=models.Distance.COSINE
            ),
        )
        print(f"Collection '{COLLECTION_NAME}' created successfully")


def clean_markdown(content: str) -> str:
    """Remove frontmatter and clean markdown content"""
    # Remove YAML frontmatter
    content = re.sub(r'^---\n.*?\n---\n', '', content, flags=re.DOTALL)

    # Remove HTML comments
    content = re.sub(r'<!--.*?-->', '', content, flags=re.DOTALL)

    # Remove import statements (MDX)
    content = re.sub(r'^import\s+.*$', '', content, flags=re.MULTILINE)

    # Remove JSX/TSX components but keep text content
    content = re.sub(r'<[A-Z][^>]*>.*?</[A-Z][^>]*>', '', content, flags=re.DOTALL)
    content = re.sub(r'<[A-Z][^/>]*/>', '', content)

    # Clean up extra whitespace
    content = re.sub(r'\n{3,}', '\n\n', content)

    return content.strip()


def extract_title(content: str, filepath: str) -> str:
    """Extract title from markdown content or filepath"""
    # Try to find H1 header
    match = re.search(r'^#\s+(.+)$', content, re.MULTILINE)
    if match:
        return match.group(1).strip()

    # Try frontmatter title
    match = re.search(r'^title:\s*["\']?(.+?)["\']?\s*$', content, re.MULTILINE)
    if match:
        return match.group(1).strip()

    # Fall back to filename
    return Path(filepath).stem.replace('-', ' ').replace('_', ' ').title()


def chunk_document(content: str, chunk_size: int = CHUNK_SIZE) -> List[str]:
    """Split document into chunks, trying to break at sentence/paragraph boundaries"""
    chunks = []
    start = 0

    while start < len(content):
        end = start + chunk_size

        if end < len(content):
            # Look for sentence ending
            for i in range(end, min(end + 200, len(content))):
                if content[i] in '.!?\n':
                    end = i + 1
                    break

        chunk_text = content[start:end].strip()
        if chunk_text and len(chunk_text) > 50:  # Only add meaningful chunks
            chunks.append(chunk_text)

        start = end

    return chunks


def generate_uuid(doc_id: str) -> str:
    """Generate consistent UUID from document ID"""
    return str(uuid.uuid5(uuid.NAMESPACE_DNS, doc_id))


def get_source_type(filepath: str) -> str:
    """Determine source type from file path"""
    path_lower = filepath.lower()

    if 'ros2' in path_lower:
        return 'ROS2_MODULE'
    elif 'gazebo' in path_lower or 'unity' in path_lower:
        return 'GAZEBO_UNITY_MODULE'
    elif 'nvidia' in path_lower or 'isaac' in path_lower:
        return 'NVIDIA_ISAAC_MODULE'
    elif 'vla' in path_lower:
        return 'VLA_MODULE'
    elif 'instructor' in path_lower:
        return 'INSTRUCTOR_GUIDE'
    elif 'lab' in path_lower:
        return 'LAB_EXERCISE'
    elif 'assessment' in path_lower or 'quiz' in path_lower or 'assignment' in path_lower:
        return 'ASSESSMENT'
    elif 'hardware' in path_lower:
        return 'HARDWARE_GUIDE'
    else:
        return 'COURSE_CONTENT'


def ingest_document(filepath: str) -> int:
    """Ingest a single document and return number of chunks added"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            raw_content = f.read()

        # Clean and process content
        content = clean_markdown(raw_content)

        if len(content) < 100:
            print(f"  Skipping {filepath} (too short)")
            return 0

        title = extract_title(raw_content, filepath)
        source_type = get_source_type(filepath)
        doc_id = Path(filepath).stem

        # Chunk the document
        chunks = chunk_document(content)

        if not chunks:
            print(f"  Skipping {filepath} (no valid chunks)")
            return 0

        # Generate embeddings for all chunks (batch for efficiency)
        print(f"  Generating embeddings for {len(chunks)} chunks...")
        embeddings_response = co.embed(
            texts=chunks,
            model=EMBEDDING_MODEL,
            input_type="search_document"
        )

        # Prepare points for Qdrant
        points = []
        for i, (chunk_text, embedding) in enumerate(zip(chunks, embeddings_response.embeddings)):
            chunk_id = f"{doc_id}_chunk_{i}"
            point_id = generate_uuid(chunk_id)

            points.append(
                models.PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "content": chunk_text,
                        "metadata": {
                            "document_id": doc_id,
                            "chunk_id": chunk_id,
                            "title": title,
                            "source_type": source_type,
                            "chunk_order": i,
                            "filepath": filepath
                        },
                        "original_doc_id": chunk_id
                    }
                )
            )

        # Upsert to Qdrant
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )

        return len(chunks)

    except Exception as e:
        print(f"  ERROR processing {filepath}: {e}")
        return 0


def main():
    """Main ingestion function"""
    print("=" * 60)
    print("RAG Document Ingestion Script")
    print("=" * 60)

    # Get docs directory
    script_dir = Path(__file__).parent
    docs_dir = script_dir.parent / "docs"

    if not docs_dir.exists():
        print(f"ERROR: Docs directory not found at {docs_dir}")
        sys.exit(1)

    print(f"\nDocs directory: {docs_dir}")

    # Setup collection
    setup_collection()

    # Find all markdown files
    md_files = list(docs_dir.glob("**/*.md"))

    # Filter out non-course content
    skip_patterns = [
        'tutorial-basics',
        'tutorial-extras',
        'translation-',
        'mdx-',
        'test-',
        'home-test'
    ]

    filtered_files = []
    for f in md_files:
        skip = False
        for pattern in skip_patterns:
            if pattern in str(f):
                skip = True
                break
        if not skip:
            filtered_files.append(f)

    print(f"\nFound {len(filtered_files)} course documents to ingest")
    print("-" * 60)

    total_chunks = 0
    successful_docs = 0

    for i, filepath in enumerate(filtered_files, 1):
        relative_path = filepath.relative_to(docs_dir)
        print(f"\n[{i}/{len(filtered_files)}] Processing: {relative_path}")

        chunks_added = ingest_document(str(filepath))

        if chunks_added > 0:
            print(f"  Added {chunks_added} chunks")
            total_chunks += chunks_added
            successful_docs += 1

    print("\n" + "=" * 60)
    print("INGESTION COMPLETE")
    print("=" * 60)
    print(f"Documents processed: {successful_docs}/{len(filtered_files)}")
    print(f"Total chunks indexed: {total_chunks}")

    # Verify
    try:
        collection_info = qdrant.get_collection(COLLECTION_NAME)
        print(f"Qdrant collection points: {collection_info.points_count}")
    except Exception as e:
        print(f"Could not verify collection: {e}")

    print("\nDone! Your chatbot should now be able to answer questions about the course content.")


if __name__ == "__main__":
    main()
