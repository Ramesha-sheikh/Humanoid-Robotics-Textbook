# Data Model: Better-Auth Signup & Signin with User Personalization

## Database Schema

### Neon DB (PostgreSQL)

#### Users Table

```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    programming_experience VARCHAR(50) NOT NULL CHECK (
        programming_experience IN ('None', 'Python', 'C++', 'Both Python and C++')
    ),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Index for fast email lookups during signin
CREATE INDEX idx_users_email ON users(email);

-- Index for analytics queries (optional)
CREATE INDEX idx_users_created_at ON users(created_at);

-- Trigger to auto-update updated_at timestamp
CREATE OR REPLACE FUNCTION update_updated_at_column()
RETURNS TRIGGER AS $$
BEGIN
    NEW.updated_at = CURRENT_TIMESTAMP;
    RETURN NEW;
END;
$$ LANGUAGE plpgsql;

CREATE TRIGGER update_users_updated_at
BEFORE UPDATE ON users
FOR EACH ROW
EXECUTE FUNCTION update_updated_at_column();
```

**Field Descriptions**:
- `id`: Unique identifier (UUID v4) for each user
- `email`: User's email address (unique constraint, used for signin)
- `password_hash`: Bcrypt hashed password (never store plaintext)
- `programming_experience`: User's self-reported programming background for personalization
- `created_at`: Account creation timestamp (UTC)
- `updated_at`: Last profile update timestamp (UTC, auto-updated)

**Constraints**:
- `email` must be unique (enforced at database level)
- `programming_experience` must be one of four allowed values
- `password_hash` must not be null

**Indexes**:
- Primary key index on `id` (default)
- Unique index on `email` (for fast lookup during signin)
- Optional index on `created_at` (for analytics)

### Qdrant Vector Database

No schema changes required. Existing collection structure:

```python
{
    "collection_name": "book",
    "vector_name": "content",
    "vector_size": 1024,  # Cohere embed-english-v3.0
    "distance": "Cosine",
    "payload_schema": {
        "text": "string",        # Original content chunk
        "url": "string",         # Source page URL
        "title": "string",       # Page title
        "language": "string"     # Programming language (if code block)
        # Note: "language" field may not exist for all records; handle gracefully
    }
}
```

**Personalization Note**: The `language` field in payload will be used for score boosting. If missing, assume the chunk is language-agnostic (conceptual content).

---

## API Models (Pydantic Schemas)

### Authentication Models

```python
from pydantic import BaseModel, EmailStr, Field
from typing import Optional, Literal
from datetime import datetime
import uuid

# ===== Request Models =====

class SignupRequest(BaseModel):
    email: EmailStr
    password: str = Field(..., min_length=8, max_length=128)
    programming_experience: Literal['None', 'Python', 'C++', 'Both Python and C++']

    class Config:
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "SecureP@ss123",
                "programming_experience": "Python"
            }
        }

class SigninRequest(BaseModel):
    email: EmailStr
    password: str

    class Config:
        json_schema_extra = {
            "example": {
                "email": "user@example.com",
                "password": "SecureP@ss123"
            }
        }

class UpdateProfileRequest(BaseModel):
    programming_experience: Literal['None', 'Python', 'C++', 'Both Python and C++']

    class Config:
        json_schema_extra = {
            "example": {
                "programming_experience": "C++"
            }
        }

# ===== Response Models =====

class UserProfile(BaseModel):
    id: uuid.UUID
    email: str
    programming_experience: str
    created_at: datetime
    updated_at: datetime

    class Config:
        json_schema_extra = {
            "example": {
                "id": "550e8400-e29b-41d4-a716-446655440000",
                "email": "user@example.com",
                "programming_experience": "Python",
                "created_at": "2025-12-27T10:30:00Z",
                "updated_at": "2025-12-27T10:30:00Z"
            }
        }

class AuthResponse(BaseModel):
    access_token: str
    token_type: str = "bearer"
    user: UserProfile

    class Config:
        json_schema_extra = {
            "example": {
                "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...",
                "token_type": "bearer",
                "user": {
                    "id": "550e8400-e29b-41d4-a716-446655440000",
                    "email": "user@example.com",
                    "programming_experience": "Python",
                    "created_at": "2025-12-27T10:30:00Z",
                    "updated_at": "2025-12-27T10:30:00Z"
                }
            }
        }

class ErrorResponse(BaseModel):
    detail: str

    class Config:
        json_schema_extra = {
            "example": {
                "detail": "Email already registered"
            }
        }
```

### JWT Token Payload

```python
# JWT payload structure (not a Pydantic model, for documentation)
{
    "sub": "550e8400-e29b-41d4-a716-446655440000",  # User ID
    "email": "user@example.com",
    "programming_experience": "Python",
    "exp": 1703721600,  # Expiration timestamp (7 days from issuance)
    "iat": 1703116800   # Issued at timestamp
}
```

**Token Specifications**:
- Algorithm: HS256 (HMAC with SHA-256)
- Secret: Stored in environment variable `JWT_SECRET_KEY`
- Expiration: 7 days (604800 seconds)
- Claims:
  - `sub`: Subject (user ID)
  - `email`: User's email
  - `programming_experience`: User's programming background (for personalization)
  - `exp`: Expiration timestamp (Unix epoch)
  - `iat`: Issued at timestamp (Unix epoch)

---

## Personalization Data Flow

### Anonymous User Request Flow

```
┌─────────┐
│ Browser │
└────┬────┘
     │ POST /stream-chat
     │ { question: "How do I create a ROS2 node?" }
     │ (No Authorization header)
     ▼
┌─────────────┐
│ FastAPI     │
│ Backend     │
└──────┬──────┘
       │ 1. Detect no auth token
       │ 2. Set user_context = None
       │ 3. Query Qdrant (no boosting)
       ▼
┌──────────────┐
│ Qdrant DB    │ Generic vector search
│ (No boost)   │ Results: balanced Python/C++ content
└──────┬───────┘
       │ 4. Return top 5 chunks
       ▼
┌─────────────┐
│ FastAPI     │ 5. Generate LLM response
│ Backend     │    (generic context)
└──────┬──────┘
       │ 6. Stream response + metadata
       │    { personalized: false }
       ▼
┌─────────┐
│ Browser │ Display generic response
└─────────┘
```

### Authenticated User Request Flow

```
┌─────────┐
│ Browser │ JWT stored in localStorage
└────┬────┘
     │ POST /stream-chat
     │ { question: "How do I create a ROS2 node?" }
     │ Authorization: Bearer eyJhbGc...
     ▼
┌─────────────┐
│ FastAPI     │
│ Backend     │
└──────┬──────┘
       │ 1. Validate JWT token
       │ 2. Extract programming_experience = "Python"
       │ 3. Set user_context = { lang: "Python" }
       │ 4. Query Qdrant with boost params
       ▼
┌──────────────┐
│ Qdrant DB    │ Vector search with score boosting
│ (Boost       │ - Chunks with "python", "rclpy" → score * 1.3
│  applied)    │ - Chunks with "c++", "rclcpp" → score * 1.0
└──────┬───────┘
       │ 5. Return top 5 chunks (Python-heavy)
       ▼
┌─────────────┐
│ FastAPI     │ 6. Generate LLM response
│ Backend     │    (Python-focused context)
└──────┬──────┘
       │ 7. Stream response + metadata
       │    { personalized: true, user_id: "550e..." }
       ▼
┌─────────┐
│ Browser │ Display personalized response
│         │ (Python code examples prioritized)
└─────────┘
```

---

## Database Connection Configuration

### Neon DB Connection String

```python
# config.py
import os
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

# Environment variable (set in .env or HF Spaces secrets)
NEON_DATABASE_URL = os.getenv("NEON_DATABASE_URL")
# Example: "postgresql://user:password@ep-cool-name-123456.us-east-2.aws.neon.tech/dbname?sslmode=require"

# SQLAlchemy engine with connection pooling
engine = create_engine(
    NEON_DATABASE_URL,
    pool_size=5,          # Minimum 5 connections (NFR-008)
    max_overflow=10,      # Allow up to 15 total connections
    pool_pre_ping=True,   # Verify connections before use (handle stale connections)
    echo=False            # Disable SQL query logging in production
)

SessionLocal = sessionmaker(autocommit=False, autoflush=False, bind=engine)

# Dependency for FastAPI route handlers
def get_db():
    db = SessionLocal()
    try:
        yield db
    finally:
        db.close()
```

---

## Data Validation Rules

### Email Validation
- Format: RFC 5322 compliant (handled by Pydantic `EmailStr`)
- Max length: 255 characters
- Must be unique in database

### Password Validation
- Min length: 8 characters
- Max length: 128 characters
- Must contain:
  - At least 1 uppercase letter
  - At least 1 lowercase letter
  - At least 1 number
- Hashing: bcrypt with 10 rounds (implemented in backend)

**Password Validation Function**:
```python
import re

def validate_password_strength(password: str) -> bool:
    """Validate password meets security requirements."""
    if len(password) < 8 or len(password) > 128:
        return False
    if not re.search(r'[A-Z]', password):  # Uppercase
        return False
    if not re.search(r'[a-z]', password):  # Lowercase
        return False
    if not re.search(r'\d', password):     # Digit
        return False
    return True
```

### Programming Experience Validation
- Must be one of: `"None"`, `"Python"`, `"C++"`, `"Both Python and C++"`
- Enforced at database level (CHECK constraint)
- Enforced at API level (Pydantic Literal type)

---

## Sample Data (for testing)

```sql
-- Test users (passwords are bcrypt hashed "TestPass123!")
INSERT INTO users (email, password_hash, programming_experience) VALUES
('alice@example.com', '$2b$10$N9qo8uLOickgx2ZMRZoMyeIjZAgcfl7p92ldGxad68LJZdL17lhWy', 'Python'),
('bob@example.com', '$2b$10$N9qo8uLOickgx2ZMRZoMyeIjZAgcfl7p92ldGxad68LJZdL17lhWy', 'C++'),
('charlie@example.com', '$2b$10$N9qo8uLOickgx2ZMRZoMyeIjZAgcfl7p92ldGxad68LJZdL17lhWy', 'None'),
('diana@example.com', '$2b$10$N9qo8uLOickgx2ZMRZoMyeIjZAgcfl7p92ldGxad68LJZdL17lhWy', 'Both Python and C++');
```

**Test Scenarios**:
- Alice: Should receive Python-focused responses
- Bob: Should receive C++-focused responses
- Charlie: Should receive conceptual, code-light responses
- Diana: Should receive balanced Python/C++ responses
