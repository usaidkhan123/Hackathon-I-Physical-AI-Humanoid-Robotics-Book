# Data Model: RAG Chatbot

**Feature**: Physical AI & Humanoid Robotics Curriculum
**Date**: 2025-12-07

This document defines the database schema for the RAG chatbot's metadata, which will be stored in Neon Serverless Postgres.

## Schema Overview

The database will consist of three main tables to manage chat interactions and link them back to the source content.

-   **`users`**: Stores a minimal amount of information to identify a unique user session.
-   **`chat_sessions`**: Represents a single conversation thread.
-   **`chat_messages`**: Stores each message within a session, including both user queries and AI responses.
-   **`message_citations`**: A join table linking AI messages back to the specific document chunks used to generate the answer.

---

## Table: `users`

Stores basic information about a user to track their sessions.

| Column      | Data Type     | Constraints              | Description                                  |
|-------------|---------------|--------------------------|----------------------------------------------|
| `id`        | `UUID`        | `PRIMARY KEY`            | Unique identifier for the user.              |
| `created_at`| `TIMESTAMPTZ` | `NOT NULL, DEFAULT NOW()`| Timestamp of user creation.                  |
| `metadata`  | `JSONB`       |                          | Optional field for extra user-related data.  |

---

## Table: `chat_sessions`

Represents a single, continuous conversation.

| Column      | Data Type     | Constraints                        | Description                                  |
|-------------|---------------|------------------------------------|----------------------------------------------|
| `id`        | `UUID`        | `PRIMARY KEY`                      | Unique identifier for the chat session.      |
| `user_id`   | `UUID`        | `FOREIGN KEY (users.id)`           | The user who initiated the session.          |
| `created_at`| `TIMESTAMPTZ` | `NOT NULL, DEFAULT NOW()`          | Timestamp of session creation.               |
| `title`     | `TEXT`        |                                    | An optional, auto-generated title for the session. |

---

## Table: `chat_messages`

Stores individual messages within a chat session.

| Column           | Data Type     | Constraints                          | Description                                         |
|------------------|---------------|--------------------------------------|-----------------------------------------------------|
| `id`             | `UUID`        | `PRIMARY KEY`                        | Unique identifier for the message.                  |
| `session_id`     | `UUID`        | `FOREIGN KEY (chat_sessions.id)`     | The session this message belongs to.                |
| `role`           | `TEXT`        | `NOT NULL, CHECK (role IN ('user', 'assistant'))` | The sender of the message.                 |
| `content`        | `TEXT`        | `NOT NULL`                           | The text content of the message.                    |
| `created_at`     | `TIMESTAMPTZ` | `NOT NULL, DEFAULT NOW()`            | Timestamp of message creation.                      |

---

## Table: `message_citations`

Links assistant messages to the source document chunks retrieved from Qdrant.

| Column           | Data Type     | Constraints                        | Description                                         |
|------------------|---------------|------------------------------------|-----------------------------------------------------|
| `message_id`     | `UUID`        | `PRIMARY KEY, FOREIGN KEY (chat_messages.id)` | The assistant message that used the citation.      |
| `chunk_id`       | `VARCHAR(255)`| `PRIMARY KEY`                        | The unique ID of the document chunk from Qdrant.    |
| `source_location`| `TEXT`        | `NOT NULL`                           | The location of the source (e.g., Module/Chapter).  |
| `retrieval_score`| `FLOAT`       |                                      | The similarity score from the vector search.        |

---

## Relationships

```
+---------+      +-----------------+      +-----------------+      +---------------------+
|  users  |      |  chat_sessions  |      |  chat_messages  |      |  message_citations  |
+---------+      +-----------------+      +-----------------+      +---------------------+
| id (PK) |----< | user_id (FK)    |      |                 |      |                     |
|         |      | id (PK)         |----< | session_id (FK) |      |                     |
|         |      |                 |      | id (PK)         |----< | message_id (FK, PK) |
|         |      |                 |      | role            |      | chunk_id (PK)       |
|         |      |                 |      | content         |      | source_location     |
+---------+      +-----------------+      +-----------------+      +---------------------+
```
