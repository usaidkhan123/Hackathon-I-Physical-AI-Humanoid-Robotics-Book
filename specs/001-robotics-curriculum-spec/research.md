# Research & Best Practices

**Feature**: Physical AI & Humanoid Robotics Curriculum
**Date**: 2025-12-07

This document outlines the key technology decisions and best practices for the project, as defined by the user prompt and project constitution.

## Decision 1: Documentation Framework

-   **Decision**: Use **Docusaurus**.
-   **Rationale**: Docusaurus is a modern, React-based static site generator designed for documentation. It provides excellent features out-of-the-box, including versioning, search, and a customizable sidebar, making it ideal for an online course book. It directly meets the project requirements.
-   **Alternatives Considered**:
    -   **MkDocs**: Simpler, Markdown-based, but less feature-rich than Docusaurus.
    -   **GitBook**: A commercial product; the goal here is an open-source-style stack.

## Decision 2: RAG Chatbot Backend

-   **Decision**: Use **FastAPI (Python)**.
-   **Rationale**: FastAPI is a high-performance, easy-to-use web framework for Python. Its asynchronous capabilities are well-suited for handling I/O-bound operations like API calls to LLMs and databases. Its automatic OpenAPI documentation generation is a significant advantage for API-driven projects.
-   **Alternatives Considered**:
    -   **Node.js/Express**: A viable alternative, but Python is the dominant language in the AI/ML ecosystem, making it a more natural fit for the project's backend.

## Decision 3: Vector Database

-   **Decision**: Use **Qdrant Cloud (Free Tier)**.
-   **Rationale**: Qdrant is a high-performance vector database specifically designed for similarity search. It offers a generous free tier on its cloud platform, which is perfect for a project of this scale. Its robust filtering and payload indexing capabilities are essential for an effective RAG implementation.
-   **Alternatives Considered**:
    -   **Pinecone**: Another popular vector database, but Qdrant's open-source nature and self-hosting options provide more flexibility long-term.
    -   **FAISS**: A library, not a managed service. It would require significant engineering effort to build a production-ready service around it.

## Decision 4: Metadata Storage

-   **Decision**: Use **Neon Serverless Postgres**.
-   **Rationale**: Neon provides a serverless, scalable PostgreSQL database. Its ability to scale to zero when not in use is highly cost-effective for development and low-traffic applications. It will be used to store user data, chat history, and citation metadata, which are relational in nature.
-   **Alternatives Considered**:
    -   **SQLite**: Simple and file-based, but not suitable for a production web service.
    -   **Standard PostgreSQL (e.g., on RDS)**: More expensive and requires manual scaling.

## Decision 5: Chatbot Orchestration

-   **Decision**: Use **OpenAI Agents & ChatKit SDKs**.
-   **Rationale**: These SDKs provide a structured framework for building multi-agent systems, as required by the constitution. They simplify the process of defining agent roles (Retriever, Safety, etc.) and managing the flow of information between them.
-   **Alternatives Considered**:
    -   **LangChain / LlamaIndex**: Powerful but can be overly complex and abstract. The OpenAI SDKs are more direct and focused for this use case.
