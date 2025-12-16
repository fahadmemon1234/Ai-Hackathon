# Quickstart Guide: Integrated RAG Chatbot

This guide provides instructions to set up and run the Integrated RAG Chatbot locally.

## Prerequisites

-   Python 3.11+
-   Node.js (LTS recommended)
-   npm or Yarn
-   Git
-   Docker (optional, for local Qdrant/Neon setup if not using cloud services)
-   Access to Google Gemini API
-   Access to Cohere Embeddings API
-   Qdrant Cloud Free Tier account (or local Docker instance)
-   Neon Serverless Postgres database (or local Postgres instance)

## 1. Clone the Repository

First, clone the project repository:

```bash
git clone [repository-url]
cd [repository-name]
```

## 2. Backend Setup (FastAPI)

1.  Navigate to the `backend/` directory:
    ```bash
    cd backend
    ```
2.  Create and activate a Python virtual environment:
    ```bash
    python -m venv venv
    ./venv/Scripts/activate # On Windows
    source venv/bin/activate # On macOS/Linux
    ```
3.  Install dependencies:
    ```bash
    pip install -r requirements.txt
    ```
4.  Configure environment variables:
    Create a `.env` file in the `backend/` directory with your API keys and database connection strings.
    ```
    # Example .env content
    OPENAI_API_KEY="your_openai_api_key"
    COHERE_API_KEY="your_cohere_api_key"
    QDRANT_URL="your_qdrant_cloud_url"
    QDRANT_API_KEY="your_qdrant_api_key"
    NEON_DATABASE_URL="your_neon_postgres_connection_string"
    GEMINI_API_KEY="your_gemini_api_key"
    ```
    *(Note: The actual environment variables will be defined during implementation.)*

5.  Run the book ingestion script:
    Before running the chatbot, you need to ingest the book's content into Qdrant.
    ```bash
    python src/ingestion_script.py
    ```
6.  Start the FastAPI application:
    ```bash
    uvicorn src.main:app --reload
    ```

## 3. Frontend Setup (Docusaurus)

1.  Navigate to the `book/` directory (from the project root):
    ```bash
    cd ../book
    ```
2.  Install Node.js dependencies:
    ```bash
    npm install
    # or yarn install
    ```
3.  Start the Docusaurus development server:
    ```bash
    npm start
    # or yarn start
    ```
    The Docusaurus site will open in your browser, and the chatbot widget should be integrated into the pages.

## 4. Using the Chatbot

-   Open your Docusaurus site (`http://localhost:3000` by default).
-   Locate the chatbot widget on any page.
-   Type your questions about the book's content.
-   To ask about selected text, highlight a passage and use the contextual chat trigger (details to be provided in the frontend implementation).

## 5. Running Tests

### Backend Tests
```bash
cd backend
pytest
```

### Frontend Tests
*(Details for frontend tests will be added during implementation)*