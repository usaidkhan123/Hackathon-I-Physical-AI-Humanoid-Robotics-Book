
import os
import re
from typing import List, Dict, Any
from bs4 import BeautifulSoup
import markdown
import google.generativeai as genai
from qdrant_client import QdrantClient, models

from config import settings

genai.configure(api_key=settings.gemini_api_key)

class RAGPipeline:
    def __init__(self):
        self.qdrant_client = QdrantClient(
            url=settings.qdrant_url,
            api_key=settings.qdrant_api_key,
        )
        self.collection_name = "physical_ai_book"
        self.embedding_size = 768
        self._ensure_collection_exists()

    def _ensure_collection_exists(self):
        try:
            self.qdrant_client.get_collection(collection_name=self.collection_name)
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

            # Simple chunking by paragraph/list item. A more advanced strategy could be used.
            # This will split by token size later if needed.
            chunks.append({
                "text": text,
                "metadata": {
                    **document["metadata"],
                    "heading": current_heading,
                }
            })
        return chunks


    def embed_and_store(self, docs_path: str):
        documents = self._get_documents_from_path(docs_path)
        all_chunks = []
        for doc in documents:
            all_chunks.extend(self._chunk_document(doc))

        texts_to_embed = [chunk["text"] for chunk in all_chunks]
        
        # The Gemini API handles batching for you.
        result = genai.embed_content(
            model="models/embedding-001",
            content=texts_to_embed,
            task_type="retrieval_document",
        )
        embeddings = result['embedding']

        self.qdrant_client.upsert(
            collection_name=self.collection_name,
            points=[
                models.PointStruct(
                    id=i, 
                    vector=embedding, 
                    payload={
                        "text": chunk["text"],
                        "source_file": chunk["metadata"]["source_file"],
                        "module": chunk["metadata"]["module"],
                        "heading": chunk["metadata"]["heading"],
                    }
                )
                for i, (embedding, chunk) in enumerate(zip(embeddings, all_chunks))
            ],
            wait=True,
        )

    def search(self, question: str, top_k: int = 5) -> List[Dict[str, Any]]:
        query_embedding = genai.embed_content(
            model="models/embedding-001",
            content=question,
            task_type="retrieval_query",
        )["embedding"]

        search_result = self.qdrant_client.search(
            collection_name=self.collection_name,
            query_vector=query_embedding,
            limit=top_k,
        )

        return [hit.payload for hit in search_result]

    def generate_answer(self, question: str, context: List[Dict[str, Any]]) -> str:
        if not context:
            return "This topic is not covered in the book yet."

        context_str = "\n\n".join([item["text"] for item in context])

        prompt = f"""
        SYSTEM:
        You are a teaching assistant for a Physical AI & Humanoid Robotics book.
        Answer ONLY using the provided context.

        CONTEXT:
        {context_str}

        USER QUESTION:
        {question}
        """

        model = genai.GenerativeModel('gemini-1.5-flash')
        response = model.generate_content(prompt)
        return response.text

rag_pipeline = RAGPipeline()