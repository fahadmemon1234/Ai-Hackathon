# Integrated RAG Chatbot Backend

This directory contains the FastAPI backend application for the Integrated RAG Chatbot. It is responsible for:
- Handling API requests for chat interactions.
- Orchestrating the RAG (Retrieval-Augmented Generation) pipeline.
- Interacting with Qdrant for vector search and Cohere for embeddings.
- Interacting with Google Gemini for generating responses.

## Setup and Running

For detailed setup instructions and how to run the backend, please refer to the main [Quickstart Guide](../../specs/1-rag-chatbot-book/quickstart.md).

### Key Endpoints

-   `POST /api/chat`: Main endpoint for chatbot interactions.

## Project Structure

```
backend/
├── src/
│   ├── main.py           # FastAPI application entry point
│   ├── api/              # API endpoints (e.g., chat.py)
│   ├── core/             # Configuration, dependencies
│   ├── services/         # Business logic (Qdrant, Agent, etc.)
│   └── models/           # Pydantic schemas
├── tests/                # Unit and integration tests
├── requirements.txt      # Python dependencies
└── .env.example          # Example environment variables
```