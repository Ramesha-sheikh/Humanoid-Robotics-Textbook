### RAG Chatbot Implementation Plan

This plan outlines the steps for developing the RAG chatbot, covering backend development with FastAPI, frontend integration with Docusaurus, and deployment considerations.

#### 1. Backend Development (FastAPI)

**1.1. Setup and Project Structure:**
*   Create a new directory for the FastAPI backend, e.g., `C:\Users\Laptronics.co\Desktop\Book\Humanoid-Robotics-Textbook\rag-backend`.
*   Initialize a Python virtual environment and install necessary packages: `fastapi`, `uvicorn`, `python-dotenv`, `openai`, `qdrant-client`, `pydantic`, `markdown-it-py`, `beautifulsoup4`, `psycopg2-binary` (if using Neon Postgres for metadata).
*   Structure the backend project with clear separation for API routes, document processing, embedding generation, and vector database interaction.

**1.2. Document Ingestion Pipeline:**

*   **Parsing Docusaurus markdown files (`my-website/docs`):**
    *   Develop a script or FastAPI endpoint to read markdown files from `my-website/docs`.
    *   Use a markdown parser (e.g., `markdown-it-py` or similar with `BeautifulSoup` for HTML conversion) to extract plain text content.
    *   Extract metadata from each document:
        *   Chapter and section information (from file path or markdown headers).
        *   Page numbers (can be simulated or based on chunking).
*   **Text chunking strategies:**
    *   Implement strategies to split the extracted text into manageable chunks suitable for embedding.
    *   Consider overlap between chunks to maintain context.
    *   Chunking should ideally respect logical boundaries like paragraphs or sections rather than arbitrary character counts.
*   **Generating embeddings (using Cohere):**
    *   Utilize the Cohere API to generate embeddings for each text chunk (e.g., `embed-english-v3.0`).
    *   Store the embedding vector along with the original text chunk.
*   **Storing embeddings and metadata in Qdrant:**
    *   Connect to Qdrant Cloud Free Tier.
    *   Create a Qdrant collection.
    *   For each chunk, store:
        *   The embedding vector.
        *   The original text chunk.
        *   Extracted metadata: `chapter`, `section`, `page` (or similar identifiers).
        *   A unique `document_id` to link chunks back to their source files.
*   **Consider using Neon Serverless Postgres for additional metadata or state if necessary:**
    *   If more complex relational metadata management or session state is required, configure a Neon Serverless Postgres database.
    *   Store document metadata (e.g., full file paths, processing status, larger document-level details) here, linking to Qdrant entries via `document_id`.

**1.3. RAG Query Pipeline:**

*   **Receiving user queries, including `highlight_context`:**
    *   The `/chat` and `/chat/stream` endpoints should accept `user_query` and an optional `highlight_context` string.
*   **Performing vector search in Qdrant:**
    *   Generate an embedding for the `user_query` (and `highlight_context` if present, potentially concatenating or weighting them).
    *   Perform a similarity search in Qdrant to retrieve the top-N most relevant text chunks.
*   **Retrieving relevant text snippets:**
    *   Extract the original text content from the retrieved Qdrant points.
*   **Prompt engineering for the LLM (Gemini):**
    *   Construct a prompt for the Gemini LLM that includes:
        *   The `user_query`.
        *   The `highlight_context` (if provided).
        *   The retrieved relevant text snippets as context.
        *   Instructions to answer based *only* on the provided context and to attribute sources.
*   **Generating responses:**
    *   Call the Gemini Generative Content API with the engineered prompt.
*   **Handling streaming responses as per `api.ts`:**
    *   For the `/chat/stream` endpoint, configure the OpenAI API call to stream responses.
    *   Yield partial responses as they are generated, matching the expected format of `api.ts`.
*   **Attributing sources:**
    *   Identify which of the retrieved chunks were primarily used by the LLM (if possible, through output analysis or by linking back to metadata).
    *   Return the `chapter`, `section`, `page` metadata for the sources along with the LLM's response.

**1.4. API Endpoints:**

*   **`/health` (GET):**
    *   Returns a simple status indicating the backend is operational.
*   **`/chat` (POST):**
    *   Input: `{\"user_query\": \"string\", \"highlight_context\": \"string | null\"}`
    *   Output: `{\"response\": \"string\", \"sources\": [{\"chapter\": \"string\", \"section\": \"string\", \"page\": \"string\"}]}`
*   **`/chat/stream` (POST):**
    *   Input: `{\"user_query\": \"string\", \"highlight_context\": \"string | null\"}`
    *   Output: Server-Sent Events (SSE) stream, with each event containing parts of the response and a final event with sources, as expected by `api.ts`.

#### 2. Frontend Integration (Docusaurus)

**2.1. Integrating `my-website/src/components/ChatBot/` components:**
*   Ensure all existing `ChatBot` components (`ChatMessage.tsx`, `ChatInput.tsx`, `ChatWindow.tsx`, `FloatingChatButton.tsx`) are correctly wired up and functional.
*   Use `api.ts` as the client for communicating with the new FastAPI backend.

**2.2. Modifying `my-website/src/components/RagChatbot.tsx` to act as the entry point and render the chat UI:**
*   `RagChatbot.tsx` should manage the state of the chat (messages, loading, error).
*   It will likely render `FloatingChatButton.tsx` and, when active, `ChatWindow.tsx`.
*   Handle user input from `ChatInput.tsx` and send it to the backend via `api.ts`.
*   Manage streaming responses from `api.ts` to update the `ChatMessage.tsx` components in real-time.

**2.3. Implementing `highlight_context` capture within Docusaurus pages to send selected text to the backend:**
*   Add an event listener to the Docusaurus content area (e.g., in `my-website/src/theme/Layout/index.tsx` or a custom Docusaurus plugin) to detect text selections.
*   When text is selected and a user initiates a chat, capture the selected text.
*   Pass this `highlight_context` string as part of the query payload to the backend `api.ts` calls.

**2.4. Displaying chat responses and attributed sources in the UI:**
*   Update `ChatMessage.tsx` or `ChatWindow.tsx` to render the LLM's response.
*   Visually display the attributed sources (chapter, section, page) as clickable links or clearly formatted text below the response.

#### 3. Deployment Considerations

**3.1. Environment variable management:**
*   **Frontend (Docusaurus/Vercel):**
    *   `NEXT_PUBLIC_API_URL`: The URL of the deployed FastAPI backend.
*   **Backend (FastAPI):**
    *   `OPENAI_API_KEY`: Your OpenAI API key.
    *   `QDRANT_URL`: Qdrant Cloud service URL.
    *   `QDRANT_API_KEY`: Qdrant Cloud API key.
    *   `NEON_POSTGRES_CONNECTION_STRING` (if used): Connection string for Neon Serverless Postgres.
*   Use `.env` files for local development and secure secrets management provided by deployment platforms (e.g., Vercel environment variables, Render secrets) for production.

**3.2. Deployment strategy for FastAPI (e.g., Vercel, Render, or a similar platform):**
*   **Option 1: Render:** Good for Python applications, supports Docker, environment variables, and custom domains.
    *   Containerize the FastAPI application using Docker.
    *   Configure `render.yaml` for deployment, specifying build and start commands.
*   **Option 2: Vercel (with a Serverless Function approach):** If the FastAPI application is small and stateless enough, it could be deployed as a serverless function on Vercel, though this might require adapting the FastAPI structure slightly.
*   **Option 3: Other platforms** like Google Cloud Run, AWS Lambda + API Gateway, etc., offer scalable serverless options.

**3.3. Deployment strategy for Docusaurus (already on Vercel):**
*   No significant changes needed for Docusaurus deployment itself, as it's already on Vercel.
*   Ensure the `NEXT_PUBLIC_API_URL` environment variable is correctly set in Vercel for the Docusaurus project to point to the deployed FastAPI backend.

### Critical Files for Implementation

1.  `my-website/src/components/ChatBot/api.ts` - Core logic for API client interaction with the backend, needs to be fully implemented with streaming and source attribution.
2.  `my-website/src/components/RagChatbot.tsx` - The main entry point for the RAG chatbot UI, will orchestrate the other chat components and handle state.
3.  `my-website/src/theme/Layout/index.tsx` (or similar global component) - Location to implement the text `highlight_context` capture mechanism.
4.  `rag-backend/main.py` (new file) - The main FastAPI application file, containing API endpoints for chat and streaming.
5.  `rag-backend/document_processor.py` (new file) - Contains logic for parsing Docusaurus markdown, chunking, and generating embeddings.