# Quickstart Guide: SpecKit-Book-RAG-Chatbot-Docusaurus

This guide provides a quick overview of how to set up and run the SpecKit Book RAG Chatbot.

## 1. Project Structure

The project follows a monorepo-like structure:

```text
spec-kit-rag-chatbot/
├── backend/                  # FastAPI backend, RAG logic, Qdrant client, embeddings
├── docusaurus-chat-plugin/   # React component for Docusaurus frontend
├── docker-compose.yml        # Docker setup for Qdrant
├── SPECIFICATION.md          # Project specification
└── README.md
```

## 2. Backend Setup (FastAPI, Agents, Qdrant, Embeddings)

### Prerequisites
- Docker (for Qdrant)
- Python 3.11+
- `pip` for Python package management

### Steps

1.  **Navigate to the `backend` directory:**
    ```bash
    cd backend
    ```

2.  **Set up environment variables:**
    Create a `.env` file based on `.env.example`. You will need to provide:
    -   `COHERE_API_KEY`: Your Cohere Free Tier API key for embeddings.
    -   `GROQ_API_KEY`: Your Groq API key (if using Groq LLM).
    -   `OPENAI_API_KEY`: Your OpenAI API key (if using OpenAI LLM).
    -   `QDRANT_HOST`: Host for Qdrant (e.g., `localhost`).
    -   `QDRANT_PORT`: Port for Qdrant (e.g., `6333`).

3.  **Install Python dependencies:**
    ```bash
    pip install -r requirements.txt
    ```

4.  **Start Qdrant using Docker:**
    Navigate back to the project root and run:
    ```bash
    docker-compose up -d
    ```

5.  **Ingest documents into Qdrant:**
    This step will parse the MDX files from your `/docs` folder, chunk them, generate embeddings using Cohere, and upload them to Qdrant.
    ```bash
    python -m app.ingestion.ingest
    ```

6.  **Run the FastAPI backend:**
    ```bash
    uvicorn app.main:app --host 0.0.0.0 --port 8000
    ```
    The backend API will be available at `http://localhost:8000`.

## 3. Frontend Integration (Docusaurus Chat Plugin)

Details for integrating the React chat component into your Docusaurus site will be provided in a separate frontend-specific guide or within the `docusaurus-chat-plugin` directory's README.

## 4. Testing the Chatbot

Once both backend and frontend are running, you can interact with the chatbot on your Docusaurus site.

-   Ask questions related to the content in your `/docs` folder.
-   Verify that responses are accurate, within the 5-8 second timeframe, and include correct source links with headings.