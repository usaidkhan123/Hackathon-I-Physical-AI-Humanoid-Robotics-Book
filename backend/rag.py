import os
import re
import time
from typing import List, Dict, Any
from bs4 import BeautifulSoup
import markdown
from groq import Groq
from qdrant_client import QdrantClient, models
from fastembed import TextEmbedding

from config import settings

# Initialize Groq client
groq_client = Groq(api_key=settings.groq_api_key)

class RAGPipeline:
    def __init__(self):
        # Use prefer_grpc=True for more reliable connection
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
            prefer_grpc=True,
            timeout=60,
        )
        self.collection_name = "physical_ai_book"
        # FastEmbed default model (BAAI/bge-small-en-v1.5) has 384 dimensions
        self.embedding_size = 384
        self.embedding_model = None  # Lazy load
        self._ensure_collection_exists()

    def _get_embedding_model(self):
        """Lazy load the embedding model to avoid slow startup"""
        if self.embedding_model is None:
            self.embedding_model = TextEmbedding(model_name="BAAI/bge-small-en-v1.5")
        return self.embedding_model

    def _ensure_collection_exists(self):
        try:
            collection_info = self.qdrant_client.get_collection(collection_name=self.collection_name)
            # Check if collection has correct dimensions, recreate if not
            if collection_info.config.params.vectors.size != self.embedding_size:
                print(f"Collection has wrong dimensions ({collection_info.config.params.vectors.size}), recreating with {self.embedding_size}")
                self.qdrant_client.recreate_collection(
                    collection_name=self.collection_name,
                    vectors_config=models.VectorParams(size=self.embedding_size, distance=models.Distance.COSINE),
                )
        except Exception:
            self.qdrant_client.recreate_collection(
                collection_name=self.collection_name,
                vectors_config=models.VectorParams(size=self.embedding_size, distance=models.Distance.COSINE),
            )

    def _get_documents_from_path(self, path: str) -> List[Dict[str, Any]]:
        documents = []
        for root, _, files in os.walk(path):
            for file in files:
                if file.endswith(".md"):
                    file_path = os.path.join(root, file)
                    with open(file_path, "r", encoding="utf-8") as f:
                        content = f.read()

                    # Extract module name from path
                    module_match = re.search(r"docs[\\/](module-\d+-[^\\/]+)", file_path)
                    module = module_match.group(1) if module_match else "general"

                    documents.append({
                        "content": content,
                        "metadata": {
                            "source_file": file_path,
                            "module": module,
                        }
                    })
        return documents

    def _chunk_document(self, document: Dict[str, Any]) -> List[Dict[str, Any]]:
        html = markdown.markdown(document["content"])
        soup = BeautifulSoup(html, "html.parser")
        chunks = []
        current_heading = ""

        for element in soup.find_all(['h1', 'h2', 'h3', 'h4', 'h5', 'h6', 'p', 'li', 'code']):
            if element.name in ['h1', 'h2', 'h3', 'h4', 'h5', 'h6']:
                current_heading = element.get_text()

            text = element.get_text().strip()
            if not text:
                continue

            chunks.append({
                "text": text,
                "metadata": {
                    **document["metadata"],
                    "heading": current_heading,
                }
            })
        return chunks

    def embed_and_store(self, docs_path: str, batch_size: int = 32):
        """Embed documents using FastEmbed (local, no API needed) and store in Qdrant"""
        documents = self._get_documents_from_path(docs_path)
        all_chunks = []
        for doc in documents:
            all_chunks.extend(self._chunk_document(doc))

        texts_to_embed = [chunk["text"] for chunk in all_chunks]
        embedding_model = self._get_embedding_model()

        # Process in batches to avoid memory issues
        all_points = []
        for batch_start in range(0, len(texts_to_embed), batch_size):
            batch_end = min(batch_start + batch_size, len(texts_to_embed))
            batch_texts = texts_to_embed[batch_start:batch_end]
            batch_chunks = all_chunks[batch_start:batch_end]

            # Embed batch
            batch_embeddings = list(embedding_model.embed(batch_texts))

            # Create points for this batch
            for i, (embedding, chunk) in enumerate(zip(batch_embeddings, batch_chunks)):
                all_points.append(
                    models.PointStruct(
                        id=batch_start + i,
                        vector=embedding.tolist(),
                        payload={
                            "text": chunk["text"],
                            "source_file": chunk["metadata"]["source_file"],
                            "module": chunk["metadata"]["module"],
                            "heading": chunk["metadata"]["heading"],
                        }
                    )
                )

        # Upsert in batches with retry logic
        upsert_batch_size = 20
        max_retries = 3
        for batch_start in range(0, len(all_points), upsert_batch_size):
            batch_end = min(batch_start + upsert_batch_size, len(all_points))
            batch_points = all_points[batch_start:batch_end]

            for attempt in range(max_retries):
                try:
                    self.qdrant_client.upsert(
                        collection_name=self.collection_name,
                        points=batch_points,
                        wait=True,
                    )
                    break  # Success, move to next batch
                except Exception as e:
                    if attempt < max_retries - 1:
                        time.sleep(2 ** attempt)  # Exponential backoff
                        continue
                    raise e

        return {"chunks_embedded": len(all_chunks)}

    def search(self, question: str, top_k: int = 5) -> List[Dict[str, Any]]:
        """Search for relevant chunks using FastEmbed for query embedding"""
        embedding_model = self._get_embedding_model()
        query_embeddings = list(embedding_model.embed([question]))
        query_embedding = query_embeddings[0].tolist()

        search_result = self.qdrant_client.query_points(
            collection_name=self.collection_name,
            query=query_embedding,
            limit=top_k,
        )

        return [hit.payload for hit in search_result.points]

    def generate_answer(self, question: str, context: List[Dict[str, Any]]) -> str:
        """Generate answer using Groq's fast LLM"""
        if not context:
            return "This topic is not covered in the book yet. Please make sure to embed the book content first by calling the /embed endpoint."

        context_str = "\n\n".join([item["text"] for item in context])

        messages = [
            {
                "role": "system",
                "content": """You are a teaching assistant for a Physical AI & Humanoid Robotics book.
Answer ONLY using the provided context. Be helpful, clear, and educational.
If the context doesn't contain relevant information, say so honestly."""
            },
            {
                "role": "user",
                "content": f"""CONTEXT:
{context_str}

QUESTION:
{question}

Please provide a helpful answer based on the context above."""
            }
        ]

        response = groq_client.chat.completions.create(
            model="llama-3.1-8b-instant",  # Fast and free
            messages=messages,
            temperature=0.7,
            max_tokens=1024,
        )

        return response.choices[0].message.content

rag_pipeline = RAGPipeline()
