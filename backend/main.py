
from fastapi import FastAPI, Depends, HTTPException
from typing import List
from fastapi.middleware.cors import CORSMiddleware
from sqlalchemy.orm import Session
import database, models, rag, config
from database import SessionLocal, engine
import uuid

database.Base.metadata.create_all(bind=engine)

app = FastAPI()

# CORS configuration - Allow all origins during development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # Allow all origins
    allow_credentials=False,  # Must be False when using "*" origins
    allow_methods=["*"],
    allow_headers=["*"],
)

def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()

@app.on_event("startup")
def on_startup():
    database.init_db()

@app.get("/")
def read_root():
    return {"status": "ok"}

@app.post("/embed", status_code=200)
def embed_content():
    """
    Embeds and indexes all book content from the 'physical-ai-book/docs' directory.
    This is a long-running process and should be triggered manually.
    """
    try:
        rag.rag_pipeline.embed_and_store("../physical-ai-book/docs")
        return {"status": "success", "message": "Content embedded successfully."}
    except Exception as e:
        import traceback
        error_trace = traceback.format_exc()
        print(f"EMBED ERROR: {error_trace}")
        raise HTTPException(status_code=500, detail=f"{str(e)}\n\nTraceback: {error_trace}")


@app.post("/search", response_model=models.SearchResponse)
def search_content(question: models.Question):
    """
    Performs a semantic search for a given question and returns the retrieved chunks.
    Useful for debugging the RAG pipeline.
    """
    try:
        results = rag.rag_pipeline.search(question.question)
        return {"results": results}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/sessions/{session_id}/messages")
def post_message(session_id: str, user_message: models.Message, db: Session = Depends(get_db)):
    """
    Handles a user message, runs the RAG pipeline, and stores the conversation.
    """
    try:
        # 1. Save user message
        db_user_message = database.Message(
            session_id=session_id,
            role="user",
            content=user_message.content,
        )
        db.add(db_user_message)
        db.commit()
        db.refresh(db_user_message)

        # 2. Run RAG pipeline
        try:
            context = rag.rag_pipeline.search(user_message.content)
            answer = rag.rag_pipeline.generate_answer(user_message.content, context)
        except Exception as rag_error:
            # If RAG fails, return a helpful message
            answer = f"I'm sorry, I encountered an issue while processing your question. Please make sure the book content has been embedded first by calling the /embed endpoint. Error: {str(rag_error)}"

        # 3. Save assistant message
        db_assistant_message = database.Message(
            session_id=session_id,
            role="assistant",
            content=answer,
        )
        db.add(db_assistant_message)
        db.commit()
        db.refresh(db_assistant_message)

        return {"role": "assistant", "content": answer}
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to process message: {str(e)}")

@app.get("/sessions/{session_id}/messages", response_model=List[models.MessageDB])
def get_messages(session_id: str, db: Session = Depends(get_db)):
    """
    Returns the full chat history for a given session.
    """
    messages = db.query(database.Message).filter(database.Message.session_id == session_id).all()
    if not messages:
        # Check if the session exists, if not create it
        session = db.query(database.Session).filter(database.Session.id == session_id).first()
        if not session:
            new_session = database.Session(id=session_id)
            db.add(new_session)
            db.commit()
    return messages

@app.post("/sessions", response_model=models.Session)
def create_session(db: Session = Depends(get_db)):
    """
    Creates a new chat session.
    """
    session_id = str(uuid.uuid4())
    new_session = database.Session(id=session_id)
    db.add(new_session)
    db.commit()
    db.refresh(new_session)
    return {"session_id": session_id}
