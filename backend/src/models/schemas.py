from typing import List, Optional
from uuid import UUID, uuid4
from pydantic import BaseModel, Field
from datetime import datetime

class BookContentChunk(BaseModel):
   id: str
   session_id: str
   role: str
   content: str
   created_at: datetime
   context_text: Optional[str] = None
   metadata_: Optional[dict] = None

class UserQuestion(BaseModel):
    session_id: Optional[str] = None
    message: str
    context_mode: str = "full_book" # "full_book" | "selected_text"
    selected_text: Optional[str] = None

class ChatbotAnswer(BaseModel):
    session_id: Optional[str] = None
    response: str
    history: List[BookContentChunk] = []

