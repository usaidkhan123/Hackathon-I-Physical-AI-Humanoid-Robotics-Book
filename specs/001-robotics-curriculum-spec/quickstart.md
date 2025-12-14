# Quickstart: Project Setup

**Feature**: Physical AI & Humanoid Robotics Curriculum
**Date**: 2025-12-07

This guide provides the step-by-step instructions to set up the development environment for this project.

## 1. Prerequisites

Ensure you have the following tools installed on your system:

-   **Git**: For version control.
-   **Node.js**: Version 18.x or higher.
-   **Python**: Version 3.11 or higher.
-   **Docker**: For running a local Qdrant instance (optional).

## 2. Core Tool Installation & Setup

### 2.1 Clone the Repository
```bash
git clone <repository_url>
cd physical-ai-book
```

### 2.2 Install Docusaurus and Dependencies
This handles the book's frontend.
```bash
npm install
```

### 2.3 Set up Python Environment for Backend
This handles the RAG chatbot's FastAPI backend.
```bash
cd rag-backend
python -m venv .venv
source .venv/bin/activate  # On Windows use `.venv\Scripts\activate`
pip install -r requirements.txt
```
*(A `requirements.txt` file will be created during implementation).*

## 3. External Services Setup

### 3.1 Qdrant Cloud
1.  Go to [Qdrant Cloud](https://cloud.qdrant.io/) and sign up for a free account.
2.  Create a new cluster in the free tier.
3.  Navigate to the "Collections" tab and create a new collection for the book content.
4.  Go to the "Access" tab and copy your Cluster URL and API Key.

### 3.2 Neon Postgres
1.  Go to [Neon](https://neon.tech/) and sign up for a free account.
2.  Create a new project.
3.  On the project dashboard, find the "Connection Details" and copy the connection string. It will look like `postgres://user:password@host:port/dbname`.

## 4. Environment Variables

Create a `.env` file in the `rag-backend` directory and add the following keys:

```
# .env

# Qdrant
QDRANT_URL=<your_qdrant_cluster_url>
QDRANT_API_KEY=<your_qdrant_api_key>

# Neon Postgres
DATABASE_URL=<your_neon_postgres_connection_string>

# OpenAI
OPENAI_API_KEY=<your_openai_api_key>
```
**NEVER commit the `.env` file to version control.**

## 5. Running the Project

### Running the Docusaurus Frontend
From the root directory:
```bash
npm run start
```
The book website will be available at `http://localhost:3000`.

### Running the FastAPI Backend
From the `rag-backend` directory:
```bash
uvicorn src.main:app --reload
```
The API will be available at `http://localhost:8000`.

## 6. SpecKit & Gemini CLI

Ensure you have the Gemini CLI installed and configured. To run governance commands, use the `/sp.*` commands as defined in the project's `.gemini/commands` directory.

Example:
```bash
/sp.tasks "Break down the work for Chapter 1"
```
