
from pydantic import BaseModel
from typing import List, Optional
import datetime

class Message(BaseModel):
    role: str
    content: str

class Conversation(BaseModel):
    messages: List[Message]

class SearchResult(BaseModel):
    text: str
    source_file: str
    module: str
    heading: str

class SearchResponse(BaseModel):
    results: List[SearchResult]

class Question(BaseModel):
    question: str

class Session(BaseModel):
    session_id: str

class MessageDB(Message):
    id: int
    session_id: str
    timestamp: datetime.datetime

    class Config:
        orm_mode = True

