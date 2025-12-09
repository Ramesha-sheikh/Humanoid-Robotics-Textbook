# Data Model: SpecKit-Book-RAG-Chatbot-Docusaurus

This document outlines the core entities and their relationships for the RAG chatbot system, based on the project specification.

## Entities

### 1. User
Represents an individual interacting with the chatbot.
- **`user_id`**: `string` (Primary Key, Unique identifier for the user)

### 2. ChatSession
Represents a conversation session between a user and the chatbot.
- **`session_id`**: `string` (Primary Key, Unique identifier for the conversation session)
- **`user_id`**: `string` (Foreign Key to User.user_id, links to the user who initiated the session)
- **`created_at`**: `datetime` (Timestamp when the session was created)
- **`updated_at`**: `datetime` (Timestamp of the last activity in the session)

### 3. ChatMessage
Represents individual messages within a chat session.
- **`message_id`**: `string` (Primary Key, Unique identifier for the message)
- **`session_id`**: `string` (Foreign Key to ChatSession.session_id, links to the parent chat session)
- **`sender`**: `string` (e.g., "user", "bot")
- **`text`**: `string` (The content of the message)
- **`timestamp`**: `datetime` (Timestamp when the message was sent)
- **`sources`**: `List<Source>` (For bot messages, a list of retrieved sources from the knowledge base)

### 4. Document
Represents an entire MDX file from the Docusaurus `/docs` folder.
- **`document_id`**: `string` (Primary Key, Unique identifier for the document/MDX file)
- **`title`**: `string` (Title of the book chapter/section from frontmatter or first heading)
- **`url`**: `string` (Relative or absolute URL to the Docusaurus page, e.g., `/docs/chapter1`)
- **`content`**: `string` (The full raw content of the MDX file)
- **`headings`**: `List<string>` (List of extracted markdown headings for metadata and semantic chunking)
- **`file_path`**: `string` (Absolute path to the MDX file on the filesystem)

### 5. Chunk
Represents a semantically coherent piece of content derived from a Document.
- **`chunk_id`**: `string` (Primary Key, Unique identifier for the chunk)
- **`document_id`**: `string` (Foreign Key to Document.document_id, links to the parent document)
- **`text`**: `string` (The textual content of the chunk, including relevant code blocks/images where applicable)
- **`start_char`**: `integer` (Starting character offset of the chunk within the original Document content)
- **`end_char`**: `integer` (Ending character offset of the chunk within the original Document content)
- **`metadata`**: `JSON` (Additional metadata, e.g., closest heading, page number, block type (code/text/image))
- **`embedding_id`**: `string` (Foreign Key to Embedding.embedding_id, links to the associated embedding vector)

### 6. Embedding
Represents the vector embedding of a Chunk.
- **`embedding_id`**: `string` (Primary Key, Unique identifier for the embedding vector)
- **`vector`**: `List<float>` (The actual embedding vector, e.g., a list of 768 floats for Cohere)
- **`model_used`**: `string` (The name of the embedding model used, e.g., "embed-multilingual-v3.0")
- **`created_at`**: `datetime` (Timestamp when the embedding was generated)

### 7. Source
Represents a retrieved source document or chunk, used in the chatbot's response.
- **`title`**: `string` (Title of the source document/chapter)
- **`url`**: `string` (URL to the specific section or page of the source)
- **`heading`**: `string` (Specific heading or subsection within the source that the answer came from)