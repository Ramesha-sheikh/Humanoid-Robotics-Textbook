# Data Model: Deploy Backend to Hugging Face Spaces

**Date**: 2025-12-25
**Feature**: 001-deploy-backend-hf
**Phase**: 1 (Design & Contracts)

## Overview

This deployment task does not introduce new data models. This document describes the existing data structures used by the API for reference during deployment validation.

## Existing Data Entities

### 1. Query (Request Model)

**Purpose**: Request body for chatbot and translation endpoints

**Attributes**:
- `question` (string): User's question or query text
- `selected_text` (string, optional): User-highlighted text for contextual queries
- `agent_id` (string, optional): ID of specialized agent to use

**Validation Rules**:
- `question` is required for `/chat` and `/stream-chat`
- `selected_text` takes priority over vector search if provided
- `agent_id` must match valid agent from agents-config.json

**Source**: `rag-backend/chatbot/app.py` line 58-61

### 2. TranslateRequest (Request Model)

**Purpose**: Request body for translation endpoint

**Attributes**:
- `page_url` (string): URL of the page to translate
- `target_language` (string): Target language code (default: "urdu")

**Validation Rules**:
- `page_url` is required
- URL can be localhost or production Vercel URL
- Backend normalizes URLs to production format for lookup

**Source**: `rag-backend/chatbot/app.py` line 192-194

### 3. Chat Response (Response Model)

**Purpose**: Response format for non-streaming chatbot queries

**Attributes**:
- `answer` (string): Generated answer from Cohere chat model
- `sources` (list[string]): List of unique URLs from retrieved chunks

**Example**:
```json
{
  "answer": "A robot is a machine...",
  "sources": [
    "https://humanoid-robotics-textbook-psi.vercel.app/docs/introduction/"
  ]
}
```

**Source**: `rag-backend/chatbot/app.py` line 128

### 4. Translation Response (Response Model)

**Purpose**: Response format for translation endpoint

**Attributes**:
- `success` (boolean): Whether translation succeeded
- `translation` (string, optional): Translated text in target language
- `source` (string): Source of translation data ("cohere_embeddings")
- `error` (string, optional): Error message if translation failed

**Example Success**:
```json
{
  "success": true,
  "translation": "01-ROS 2 معرفی\n\n...",
  "source": "cohere_embeddings"
}
```

**Example Error**:
```json
{
  "success": false,
  "error": "No content found for page"
}
```

**Source**: `rag-backend/chatbot/app.py` line 292-300

### 5. Health Response (Response Model)

**Purpose**: Response format for health check endpoint

**Attributes**:
- `status` (string): Health status ("ok")
- `version` (string): API version number

**Example**:
```json
{
  "status": "ok",
  "version": "1.0"
}
```

**Source**: `rag-backend/chatbot/app.py` line 85-86

## Configuration Data

### Environment Variables (Deployment Secrets)

**Purpose**: Runtime configuration stored in HF Spaces secrets

**Variables**:
- `COHERE_API_KEY` (string): Cohere API authentication key
- `QDRANT_URL` (string): Qdrant cloud instance URL
- `QDRANT_API_KEY` (string): Qdrant authentication key
- `COLLECTION_NAME` (string): Vector collection name (default: "book")

**Validation**:
- All variables must be set before app startup
- Missing variables will cause connection failures
- Documented in `.env.example` file (to be created)

**Source**: `rag-backend/chatbot/config.py`

## Vector Store Data

### Embedded Book Content (Qdrant Collection)

**Purpose**: Pre-embedded book chapters for RAG retrieval

**Structure**:
- **Collection**: "book"
- **Vector Dimension**: 1024 (Cohere embed-english-v3.0)
- **Payload**:
  - `chunk_text` (string): Text content of chunk
  - `url` (string): Source page URL
- **Total Points**: 280 embedded chunks

**Note**: This data already exists in Qdrant Cloud and does not need to be modified during deployment.

**Source**: External (Qdrant Cloud)

## State Management

### No Persistent State

This backend is **stateless** - no sessions, no user accounts, no database writes. All data is:
- **Read-only** from Qdrant (vector search)
- **Transient** API calls to Cohere (chat/embeddings)
- **Ephemeral** in-memory processing

This simplifies deployment - no database migrations, no state to backup.

## Data Flow

```
User Request
    ↓
Frontend (Vercel)
    ↓ HTTPS/CORS
Backend (HF Spaces)
    ↓ API Call
Cohere API (embed query)
    ↓ Vector
Qdrant Cloud (search)
    ↓ Chunks
Backend (format response)
    ↓ API Call
Cohere API (generate answer)
    ↓ Response
Backend (return JSON)
    ↓ HTTPS
Frontend (display)
```

## Deployment Impact

### No Data Migration Required

- No database schema changes
- No data seeding needed
- Existing Qdrant collection remains unchanged
- All API contracts remain backward-compatible

### Validation Checklist

After deployment, verify:
- [ ] Chat endpoint returns valid JSON with `answer` and `sources`
- [ ] Translation endpoint returns valid JSON with `success` and `translation`
- [ ] Health endpoint returns `status: "ok"`
- [ ] Error responses include user-friendly messages
- [ ] CORS headers present in all responses

## References

- Backend source code: `rag-backend/chatbot/app.py`
- Configuration: `rag-backend/chatbot/config.py`
- Pydantic models defined inline in app.py (lines 58-61, 192-194)
