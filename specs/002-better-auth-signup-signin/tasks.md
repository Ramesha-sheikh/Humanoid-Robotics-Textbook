# Implementation Tasks: Better-Auth Signup & Signin with User Personalization

**Feature Branch**: `002-better-auth-signup-signin`
**Created**: 2025-12-27
**Status**: Ready for Implementation
**Reference**: [spec.md](./spec.md), [plan.md](./plan.md), [data-model.md](./data-model.md)

---

## Task Dependency Graph

```
T-001 (Environment Setup)
  ↓
T-002 (Database Schema) ← T-003 (SQLAlchemy Models)
  ↓
T-004 (Password Hashing) ← T-005 (JWT Utils)
  ↓
T-006 (Signup Endpoint) ← T-007 (Signin Endpoint) ← T-008 (Get Profile) ← T-009 (Update Profile)
  ↓
T-010 (JWT Middleware)
  ↓
T-011 (Personalization Logic) ← T-012 (Enhanced Chatbot Endpoints)
  ↓
T-013 (Frontend Auth Context) ← T-014 (Signup Modal) ← T-015 (Signin Modal)
  ↓
T-016 (Profile Settings Modal) ← T-017 (Header Auth Button) ← T-018 (API Client Updates)
  ↓
T-019 (Unit Tests) ← T-020 (Integration Tests)
  ↓
T-021 (Deployment Setup) ← T-022 (Production Testing)
```

---

## Phase 1: Backend Foundation (Priority: P0 - Critical Path)

### T-001: Environment Setup and Dependencies
**Priority**: P0 (Blocking)
**Estimated Effort**: 30 minutes
**Assignee**: Backend Developer
**Dependencies**: None

**Description**:
Set up development environment with required Python packages and database infrastructure.

**Acceptance Criteria**:
- [X] Neon DB account created and database provisioned
- [X] Connection string saved in `.env` file (not committed to Git)
- [X] JWT secret generated (`openssl rand -hex 32`) and saved in `.env`
- [X] Python dependencies added to `requirements.txt`:
  - `sqlalchemy>=2.0.0`
  - `psycopg2-binary>=2.9.0`
  - `pyjwt>=2.8.0`
  - `bcrypt>=4.0.0`
  - `python-multipart>=0.0.6`
  - `alembic>=1.12.0`
- [X] Dependencies installed: `pip install -r requirements.txt`
- [ ] `.env` file added to `.gitignore`

**Test Cases**:
1. Run `pip list | grep sqlalchemy` → Shows SQLAlchemy 2.0+
2. Run `pip list | grep pyjwt` → Shows PyJWT 2.8+
3. Verify `.env` file not tracked: `git status` → `.env` not listed

**Files Created**:
- `rag-backend/chatbot/.env` (not committed)
- `rag-backend/chatbot/requirements.txt` (modified)

**Reference**: [plan.md - Phase 1](./plan.md#phase-1-backend-foundation-priority-p1)

---

### T-002: Create Database Schema with Alembic
**Priority**: P0 (Blocking)
**Estimated Effort**: 1 hour
**Assignee**: Backend Developer
**Dependencies**: T-001

**Description**:
Initialize Alembic for database migrations and create the `users` table schema.

**Acceptance Criteria**:
- [ ] Alembic initialized: `alembic init alembic` (run from `rag-backend/chatbot/`)
- [ ] `alembic.ini` configured with `sqlalchemy.url` pointing to Neon DB
- [ ] Migration script created: `alembic revision -m "Create users table"`
- [ ] Migration includes:
  - `users` table with UUID primary key
  - Unique constraint on `email`
  - Check constraint on `programming_experience` (enum validation)
  - Indexes on `email` and `created_at`
  - Auto-update trigger for `updated_at`
- [ ] Migration applied: `alembic upgrade head`
- [ ] Verify table exists: Connect to Neon DB and run `\dt` → Shows `users` table

**Test Cases**:
1. Run `alembic current` → Shows migration applied
2. Query Neon DB: `SELECT * FROM users LIMIT 0;` → Returns empty result (table exists)
3. Test unique constraint: Insert duplicate email → Raises `IntegrityError`
4. Test check constraint: Insert invalid programming_experience → Raises `CheckViolation`

**Files Created**:
- `rag-backend/chatbot/alembic/` (directory with migration scripts)
- `rag-backend/chatbot/alembic.ini`
- `rag-backend/chatbot/alembic/env.py` (configured with DB URL)
- `rag-backend/chatbot/alembic/versions/XXXX_create_users_table.py`

**SQL Schema** (for reference):
```sql
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    password_hash VARCHAR(255) NOT NULL,
    programming_experience VARCHAR(50) NOT NULL
        CHECK (programming_experience IN ('None', 'Python', 'C++', 'Both Python and C++')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

CREATE INDEX idx_users_email ON users(email);
CREATE INDEX idx_users_created_at ON users(created_at);

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

**Reference**: [data-model.md - Database Schema](./data-model.md#database-schema)

---

### T-003: Create SQLAlchemy Models
**Priority**: P0 (Blocking)
**Estimated Effort**: 45 minutes
**Assignee**: Backend Developer
**Dependencies**: T-002

**Description**:
Define SQLAlchemy ORM models for the `users` table with Pydantic schemas for validation.

**Acceptance Criteria**:
- [ ] File created: `rag-backend/chatbot/models.py`
- [ ] `User` SQLAlchemy model defined with:
  - UUID primary key (auto-generated)
  - String fields: `email`, `password_hash`, `programming_experience`
  - Timestamp fields: `created_at`, `updated_at`
  - Unique constraint on `email`
- [ ] File created: `rag-backend/chatbot/schemas.py`
- [ ] Pydantic schemas defined:
  - `SignupRequest` (email, password, programming_experience)
  - `SigninRequest` (email, password)
  - `UpdateProfileRequest` (programming_experience)
  - `UserProfile` (id, email, programming_experience, timestamps)
  - `AuthResponse` (access_token, token_type, user)
  - `ErrorResponse` (detail)
- [ ] Email validation uses `EmailStr` from Pydantic
- [ ] Password field has `min_length=8, max_length=128`
- [ ] Programming experience uses `Literal['None', 'Python', 'C++', 'Both Python and C++']`

**Test Cases**:
1. Create User instance: `user = User(email="test@example.com", ...)` → No errors
2. Validate SignupRequest: `SignupRequest(email="invalid", ...)` → Raises `ValidationError`
3. Validate password length: `SignupRequest(password="short", ...)` → Raises `ValidationError`
4. Validate programming_experience: `SignupRequest(programming_experience="Java", ...)` → Raises `ValidationError`

**Files Created**:
- `rag-backend/chatbot/models.py` (~50 LOC)
- `rag-backend/chatbot/schemas.py` (~100 LOC)

**Code Template** (models.py):
```python
from sqlalchemy import Column, String, DateTime
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.sql import func
import uuid

from database import Base

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    email = Column(String(255), unique=True, nullable=False, index=True)
    password_hash = Column(String(255), nullable=False)
    programming_experience = Column(String(50), nullable=False)
    created_at = Column(DateTime(timezone=True), server_default=func.now())
    updated_at = Column(DateTime(timezone=True), server_default=func.now(), onupdate=func.now())
```

**Reference**: [data-model.md - API Models](./data-model.md#api-models-pydantic-schemas)

---

### T-004: Implement Password Hashing Utilities
**Priority**: P0 (Blocking)
**Estimated Effort**: 30 minutes
**Assignee**: Backend Developer
**Dependencies**: T-001

**Description**:
Create utility functions for password hashing and verification using bcrypt with 10 rounds.

**Acceptance Criteria**:
- [ ] File created: `rag-backend/chatbot/auth_utils.py`
- [ ] Function `hash_password(password: str) -> str` implemented:
  - Uses `bcrypt.hashpw()` with 10 rounds (2^10 iterations)
  - Returns hashed password as string
- [ ] Function `verify_password(plain_password: str, hashed_password: str) -> bool` implemented:
  - Uses `bcrypt.checkpw()` with constant-time comparison
  - Returns True if password matches, False otherwise
- [ ] Function `validate_password_strength(password: str) -> bool` implemented:
  - Checks min 8 chars, max 128 chars
  - Requires at least 1 uppercase, 1 lowercase, 1 digit
  - Raises `ValueError` with helpful message if validation fails

**Test Cases**:
1. Hash password: `hash_password("TestPass123!")` → Returns bcrypt hash starting with `$2b$10$`
2. Verify correct password: `verify_password("TestPass123!", hash)` → Returns `True`
3. Verify wrong password: `verify_password("WrongPass", hash)` → Returns `False`
4. Validate strong password: `validate_password_strength("SecureP@ss123")` → Returns `True`
5. Validate weak password: `validate_password_strength("weak")` → Raises `ValueError`
6. Timing attack test: Measure time for correct vs wrong password → Difference < 5ms (constant-time)

**Files Created**:
- `rag-backend/chatbot/auth_utils.py` (~50 LOC)

**Code Template**:
```python
import bcrypt
import re

def hash_password(password: str) -> str:
    """Hash password with bcrypt (10 rounds)."""
    return bcrypt.hashpw(password.encode('utf-8'), bcrypt.gensalt(rounds=10)).decode('utf-8')

def verify_password(plain_password: str, hashed_password: str) -> bool:
    """Verify password with constant-time comparison."""
    return bcrypt.checkpw(plain_password.encode('utf-8'), hashed_password.encode('utf-8'))

def validate_password_strength(password: str) -> bool:
    """Validate password meets security requirements."""
    if len(password) < 8 or len(password) > 128:
        raise ValueError("Password must be 8-128 characters")
    if not re.search(r'[A-Z]', password):
        raise ValueError("Password must contain at least one uppercase letter")
    if not re.search(r'[a-z]', password):
        raise ValueError("Password must contain at least one lowercase letter")
    if not re.search(r'\d', password):
        raise ValueError("Password must contain at least one digit")
    return True
```

**Reference**: [plan.md - ADR-001](./plan.md#decision-1-use-custom-jwt-auth-with-fastapi-not-better-auth), [data-model.md - Password Validation](./data-model.md#password-validation)

---

### T-005: Implement JWT Token Utilities
**Priority**: P0 (Blocking)
**Estimated Effort**: 1 hour
**Assignee**: Backend Developer
**Dependencies**: T-001

**Description**:
Create utility functions for generating and validating JWT tokens with 7-day expiration.

**Acceptance Criteria**:
- [ ] Functions added to `rag-backend/chatbot/auth_utils.py`:
- [ ] Function `create_access_token(user_id: UUID, email: str, programming_experience: str) -> str`:
  - Generates JWT with HS256 algorithm
  - Payload includes: `sub` (user_id), `email`, `programming_experience`, `exp` (7 days), `iat`
  - Uses `JWT_SECRET_KEY` from environment variable
  - Returns encoded JWT string
- [ ] Function `decode_access_token(token: str) -> dict`:
  - Decodes and validates JWT signature
  - Checks expiration (`exp` claim)
  - Raises `jwt.ExpiredSignatureError` if expired
  - Raises `jwt.InvalidTokenError` if invalid signature
  - Returns payload dict with user info
- [ ] JWT_SECRET_KEY loaded from environment with validation (min 32 chars)

**Test Cases**:
1. Create token: `token = create_access_token(user_id, email, prog_exp)` → Returns JWT string
2. Decode valid token: `decode_access_token(token)` → Returns payload with user_id, email, prog_exp
3. Decode expired token: Create token with `exp` in past → Raises `ExpiredSignatureError`
4. Decode invalid signature: Modify token string → Raises `InvalidTokenError`
5. Verify expiration: Token created now expires in 7 days (604800 seconds)
6. Verify payload: Decoded token contains correct `sub`, `email`, `programming_experience`

**Files Modified**:
- `rag-backend/chatbot/auth_utils.py` (+50 LOC)

**Code Template**:
```python
import jwt
import os
from datetime import datetime, timedelta
from uuid import UUID

JWT_SECRET_KEY = os.getenv("JWT_SECRET_KEY")
if not JWT_SECRET_KEY or len(JWT_SECRET_KEY) < 32:
    raise ValueError("JWT_SECRET_KEY must be set and at least 32 characters")

JWT_ALGORITHM = "HS256"
JWT_EXPIRATION_DAYS = 7

def create_access_token(user_id: UUID, email: str, programming_experience: str) -> str:
    """Generate JWT token with 7-day expiration."""
    payload = {
        "sub": str(user_id),
        "email": email,
        "programming_experience": programming_experience,
        "exp": datetime.utcnow() + timedelta(days=JWT_EXPIRATION_DAYS),
        "iat": datetime.utcnow()
    }
    return jwt.encode(payload, JWT_SECRET_KEY, algorithm=JWT_ALGORITHM)

def decode_access_token(token: str) -> dict:
    """Decode and validate JWT token."""
    try:
        payload = jwt.decode(token, JWT_SECRET_KEY, algorithms=[JWT_ALGORITHM])
        return payload
    except jwt.ExpiredSignatureError:
        raise  # Token expired
    except jwt.InvalidTokenError:
        raise  # Invalid signature or malformed token
```

**Reference**: [plan.md - ADR-002](./plan.md#decision-2-jwt-tokens-with-7-day-expiration-not-session-based-auth), [data-model.md - JWT Token Payload](./data-model.md#jwt-token-payload)

---

### T-006: Implement Signup Endpoint (POST /auth/signup)
**Priority**: P0 (Blocking)
**Estimated Effort**: 1.5 hours
**Assignee**: Backend Developer
**Dependencies**: T-003, T-004, T-005

**Description**:
Create POST /auth/signup endpoint that registers new users with email, password, and programming experience.

**Acceptance Criteria**:
- [ ] Endpoint added to `rag-backend/chatbot/app.py` (or new `rag-backend/chatbot/routes/auth.py`)
- [ ] Route: `POST /auth/signup`
- [ ] Request body: `SignupRequest` (email, password, programming_experience)
- [ ] Response: `AuthResponse` (access_token, token_type, user) with status 201
- [ ] Logic:
  1. Validate password strength (call `validate_password_strength()`)
  2. Check if email already exists → Return 409 Conflict if duplicate
  3. Hash password with bcrypt (call `hash_password()`)
  4. Create user record in database
  5. Generate JWT token (call `create_access_token()`)
  6. Return token + user profile (exclude password_hash)
- [ ] Error handling:
  - 422: Validation error (invalid email, weak password, invalid programming_experience)
  - 409: Duplicate email
  - 500: Database connection error

**Test Cases**:
1. Valid signup: Send valid SignupRequest → Returns 201 with token and user profile
2. Duplicate email: Signup twice with same email → Second request returns 409
3. Invalid email: Send invalid email format → Returns 422 with error detail
4. Weak password: Send password "weak" → Returns 422 with error message
5. Invalid programming_experience: Send "Java" → Returns 422 with error message
6. Database down: Disconnect DB, send signup → Returns 500 with error message
7. Token verification: Decode returned token → Contains correct user_id, email, programming_experience

**Files Created/Modified**:
- `rag-backend/chatbot/routes/auth.py` (~80 LOC, new file)
- `rag-backend/chatbot/app.py` (modified to include auth routes)

**Code Template**:
```python
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from sqlalchemy.exc import IntegrityError

from database import get_db
from models import User
from schemas import SignupRequest, AuthResponse, UserProfile
from auth_utils import hash_password, validate_password_strength, create_access_token

router = APIRouter(prefix="/auth", tags=["authentication"])

@router.post("/signup", response_model=AuthResponse, status_code=status.HTTP_201_CREATED)
def signup(request: SignupRequest, db: Session = Depends(get_db)):
    # Validate password strength
    try:
        validate_password_strength(request.password)
    except ValueError as e:
        raise HTTPException(status_code=422, detail=str(e))

    # Check duplicate email
    existing_user = db.query(User).filter(User.email == request.email).first()
    if existing_user:
        raise HTTPException(status_code=409, detail="Email already registered")

    # Create user
    hashed_password = hash_password(request.password)
    new_user = User(
        email=request.email,
        password_hash=hashed_password,
        programming_experience=request.programming_experience
    )

    try:
        db.add(new_user)
        db.commit()
        db.refresh(new_user)
    except IntegrityError:
        db.rollback()
        raise HTTPException(status_code=409, detail="Email already registered")

    # Generate token
    access_token = create_access_token(
        user_id=new_user.id,
        email=new_user.email,
        programming_experience=new_user.programming_experience
    )

    return AuthResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserProfile.from_orm(new_user)
    )
```

**Reference**: [contracts/api.md - POST /auth/signup](./contracts/api.md#1-post-authsignup)

---

### T-007: Implement Signin Endpoint (POST /auth/signin)
**Priority**: P0 (Blocking)
**Estimated Effort**: 1 hour
**Assignee**: Backend Developer
**Dependencies**: T-003, T-004, T-005

**Description**:
Create POST /auth/signin endpoint that authenticates users and returns JWT token.

**Acceptance Criteria**:
- [ ] Route: `POST /auth/signin` added to auth router
- [ ] Request body: `SigninRequest` (email, password)
- [ ] Response: `AuthResponse` (access_token, token_type, user) with status 200
- [ ] Logic:
  1. Query user by email
  2. If user not found → Return 401 Unauthorized ("Invalid email or password")
  3. Verify password with constant-time comparison (call `verify_password()`)
  4. If password incorrect → Return 401 Unauthorized ("Invalid email or password")
  5. Generate JWT token (call `create_access_token()`)
  6. Return token + user profile
- [ ] Security: Use same error message for "user not found" and "wrong password" (prevent user enumeration)

**Test Cases**:
1. Valid signin: Send correct email + password → Returns 200 with token
2. Wrong password: Send correct email + wrong password → Returns 401 with "Invalid email or password"
3. Non-existent email: Send unregistered email → Returns 401 with "Invalid email or password"
4. Invalid email format: Send malformed email → Returns 422
5. Timing attack test: Measure time for valid email + wrong password vs invalid email → Difference < 50ms
6. Token verification: Decode returned token → Contains correct user info

**Files Modified**:
- `rag-backend/chatbot/routes/auth.py` (+40 LOC)

**Code Template**:
```python
@router.post("/signin", response_model=AuthResponse)
def signin(request: SigninRequest, db: Session = Depends(get_db)):
    # Query user
    user = db.query(User).filter(User.email == request.email).first()

    # Constant-time check (prevent timing attacks)
    if not user or not verify_password(request.password, user.password_hash):
        raise HTTPException(status_code=401, detail="Invalid email or password")

    # Generate token
    access_token = create_access_token(
        user_id=user.id,
        email=user.email,
        programming_experience=user.programming_experience
    )

    return AuthResponse(
        access_token=access_token,
        token_type="bearer",
        user=UserProfile.from_orm(user)
    )
```

**Reference**: [contracts/api.md - POST /auth/signin](./contracts/api.md#2-post-authsignin)

---

### T-008: Implement Get Profile Endpoint (GET /auth/me)
**Priority**: P1
**Estimated Effort**: 45 minutes
**Assignee**: Backend Developer
**Dependencies**: T-010

**Description**:
Create GET /auth/me endpoint that returns current authenticated user's profile.

**Acceptance Criteria**:
- [ ] Route: `GET /auth/me` added to auth router
- [ ] Requires authentication (uses JWT middleware from T-010)
- [ ] Response: `UserProfile` with status 200
- [ ] Logic:
  1. Extract user info from JWT token (handled by middleware)
  2. Query user from database by ID
  3. Return user profile (exclude password_hash)
- [ ] Error handling:
  - 401: Missing or invalid token
  - 404: User not found in database (edge case: user deleted after token issued)

**Test Cases**:
1. Valid token: Send GET /auth/me with valid Authorization header → Returns 200 with user profile
2. Missing token: Send GET /auth/me without Authorization header → Returns 401
3. Invalid token: Send GET /auth/me with malformed token → Returns 401
4. Expired token: Send GET /auth/me with expired token → Returns 401 with "Token has expired"
5. User deleted: Issue token, delete user, call endpoint → Returns 404

**Files Modified**:
- `rag-backend/chatbot/routes/auth.py` (+30 LOC)

**Code Template**:
```python
from fastapi import Depends
from auth_utils import decode_access_token

def get_current_user(token: str = Depends(oauth2_scheme), db: Session = Depends(get_db)) -> User:
    """Dependency to extract current user from JWT token."""
    try:
        payload = decode_access_token(token)
        user_id = payload.get("sub")
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token has expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid authentication token")

    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return user

@router.get("/me", response_model=UserProfile)
def get_profile(current_user: User = Depends(get_current_user)):
    return UserProfile.from_orm(current_user)
```

**Reference**: [contracts/api.md - GET /auth/me](./contracts/api.md#3-get-authme)

---

### T-009: Implement Update Profile Endpoint (PUT /auth/me)
**Priority**: P2
**Estimated Effort**: 30 minutes
**Assignee**: Backend Developer
**Dependencies**: T-010

**Description**:
Create PUT /auth/me endpoint that allows users to update their programming experience.

**Acceptance Criteria**:
- [ ] Route: `PUT /auth/me` added to auth router
- [ ] Requires authentication (uses JWT middleware)
- [ ] Request body: `UpdateProfileRequest` (programming_experience)
- [ ] Response: `UserProfile` (updated) with status 200
- [ ] Logic:
  1. Extract current user from JWT token
  2. Validate programming_experience (Pydantic handles this)
  3. Update user record in database
  4. Return updated user profile
- [ ] Auto-update `updated_at` timestamp (handled by trigger)

**Test Cases**:
1. Valid update: Send PUT /auth/me with valid programming_experience → Returns 200 with updated profile
2. Invalid value: Send PUT /auth/me with "Java" → Returns 422
3. Idempotent: Update to "Python" twice → Both return 200, second has same result
4. Verify persistence: Update to "C++", call GET /auth/me → Returns "C++"
5. Timestamp updated: Verify `updated_at` changed after update

**Files Modified**:
- `rag-backend/chatbot/routes/auth.py` (+25 LOC)

**Code Template**:
```python
@router.put("/me", response_model=UserProfile)
def update_profile(
    request: UpdateProfileRequest,
    current_user: User = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    current_user.programming_experience = request.programming_experience
    db.commit()
    db.refresh(current_user)

    return UserProfile.from_orm(current_user)
```

**Reference**: [contracts/api.md - PUT /auth/me](./contracts/api.md#4-put-authme)

---

### T-010: Implement JWT Authentication Middleware
**Priority**: P0 (Blocking)
**Estimated Effort**: 1 hour
**Assignee**: Backend Developer
**Dependencies**: T-005

**Description**:
Create FastAPI dependency for extracting and validating JWT tokens from Authorization header.

**Acceptance Criteria**:
- [ ] File created: `rag-backend/chatbot/middleware.py` (or add to `auth_utils.py`)
- [ ] OAuth2 password bearer scheme configured: `oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/signin")`
- [ ] Dependency function `get_current_user_optional()` created:
  - Extracts token from `Authorization: Bearer <token>` header
  - Returns `None` if header missing (for optional auth endpoints like chatbot)
  - Decodes JWT and returns user context if valid
  - Raises `HTTPException(401)` if token invalid/expired (for required auth endpoints)
- [ ] Dependency function `get_current_user()` created:
  - Same as optional version but raises 401 if token missing
  - Used for endpoints requiring authentication (GET/PUT /auth/me)

**Test Cases**:
1. Valid token: Call endpoint with valid Authorization header → Extracts user context correctly
2. Missing token (optional): Call chatbot endpoint without header → Returns None, endpoint works anonymously
3. Missing token (required): Call /auth/me without header → Returns 401 "Not authenticated"
4. Invalid token: Send malformed token → Returns 401 "Invalid authentication token"
5. Expired token: Send expired token → Returns 401 "Token has expired"

**Files Created**:
- `rag-backend/chatbot/middleware.py` (~60 LOC) OR add to `auth_utils.py`

**Code Template**:
```python
from fastapi import Depends, HTTPException, status
from fastapi.security import OAuth2PasswordBearer
from typing import Optional
import jwt

from auth_utils import decode_access_token
from database import get_db
from models import User

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/signin", auto_error=False)

def get_current_user_optional(
    token: Optional[str] = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
) -> Optional[dict]:
    """Extract user context from JWT token (optional - returns None if missing)."""
    if not token:
        return None

    try:
        payload = decode_access_token(token)
        return payload  # Contains user_id, email, programming_experience
    except (jwt.ExpiredSignatureError, jwt.InvalidTokenError):
        return None  # Invalid token treated as anonymous user

def get_current_user(
    token: str = Depends(oauth2_scheme),
    db: Session = Depends(get_db)
) -> User:
    """Extract user from JWT token (required - raises 401 if missing/invalid)."""
    if not token:
        raise HTTPException(status_code=401, detail="Not authenticated")

    try:
        payload = decode_access_token(token)
        user_id = payload.get("sub")
    except jwt.ExpiredSignatureError:
        raise HTTPException(status_code=401, detail="Token has expired")
    except jwt.InvalidTokenError:
        raise HTTPException(status_code=401, detail="Invalid authentication token")

    user = db.query(User).filter(User.id == user_id).first()
    if not user:
        raise HTTPException(status_code=404, detail="User not found")

    return user
```

**Reference**: [plan.md - JWT Validation](./plan.md#decision-1-use-custom-jwt-auth-with-fastapi-not-better-auth)

---

## Phase 2: Personalization Engine (Priority: P0 - Critical Path)

### T-011: Implement Score Boosting Algorithm
**Priority**: P0 (Blocking)
**Estimated Effort**: 2 hours
**Assignee**: Backend Developer
**Dependencies**: T-010

**Description**:
Modify Qdrant query logic to boost relevance scores based on user's programming experience.

**Acceptance Criteria**:
- [ ] Function created: `apply_score_boosting(results: List, user_context: Optional[dict]) -> List`
- [ ] Input: Qdrant search results (top 20 chunks) + user context from JWT
- [ ] Logic:
  - Extract `programming_experience` from user context
  - For each result chunk:
    - Get `language` field from payload metadata (if exists)
    - Apply boost factor to similarity score:
      - Python user + "python"/"rclpy"/".py" in language/text → score * 1.3
      - C++ user + "c++"/"rclcpp"/".cpp"/".hpp" in language/text → score * 1.3
      - None user + "explanation"/"overview"/"concept" in text → score * 1.2
      - Both user → no boost (score * 1.0)
  - Re-sort results by boosted score (descending)
  - Return top 5 boosted results
- [ ] Graceful degradation: If `language` metadata missing, treat as language-agnostic (no boost)
- [ ] Performance: <50ms overhead for 20 results

**Test Cases**:
1. Python user: Query "ROS2 nodes" → Top result contains Python (rclpy) code
2. C++ user: Query "ROS2 nodes" → Top result contains C++ (rclcpp) code
3. None user: Query "ROS2 nodes" → Top result is conceptual explanation
4. Both user: Query "ROS2 nodes" → Top results are balanced Python/C++
5. Missing metadata: Query with chunks lacking `language` field → No errors, graceful handling
6. Performance: Measure boost function time → <50ms for 20 results

**Files Modified**:
- `rag-backend/chatbot/app.py` (add `apply_score_boosting()` function, ~40 LOC)

**Code Template**:
```python
def apply_score_boosting(
    results: List[dict],
    user_context: Optional[dict]
) -> List[dict]:
    """Apply relevance score boosting based on user's programming experience."""
    if not user_context:
        return results  # Anonymous user, no boosting

    programming_experience = user_context.get("programming_experience", "None")

    # Define boost keywords
    boost_keywords = {
        "Python": ["python", "rclpy", ".py", "#!/usr/bin/env python"],
        "C++": ["c++", "cpp", "rclcpp", ".cpp", ".hpp", "#include"],
        "None": ["explanation", "overview", "concept", "introduction", "understanding"]
    }

    # Apply boost
    for result in results:
        score = result.get("score", 0.0)
        text = result.get("payload", {}).get("text", "").lower()
        language = result.get("payload", {}).get("language", "").lower()

        boost_factor = 1.0

        if programming_experience == "Python":
            if any(kw in text or kw in language for kw in boost_keywords["Python"]):
                boost_factor = 1.3
        elif programming_experience == "C++":
            if any(kw in text or kw in language for kw in boost_keywords["C++"]):
                boost_factor = 1.3
        elif programming_experience == "None":
            if any(kw in text for kw in boost_keywords["None"]):
                boost_factor = 1.2
        # "Both Python and C++" → boost_factor = 1.0 (no boosting)

        result["boosted_score"] = score * boost_factor

    # Sort by boosted score
    results.sort(key=lambda x: x.get("boosted_score", 0.0), reverse=True)

    return results[:5]  # Return top 5 boosted results
```

**Reference**: [plan.md - ADR-003](./plan.md#decision-3-relevance-score-boosting-not-pre-filtering-or-llm-re-ranking), [ADR-003](../adr/003-personalization-score-boosting.md)

---

### T-012: Enhance Chatbot Endpoints with Optional Auth
**Priority**: P0 (Blocking)
**Estimated Effort**: 1.5 hours
**Assignee**: Backend Developer
**Dependencies**: T-011

**Description**:
Modify `/chat` and `/stream-chat` endpoints to accept optional Authorization header and apply personalization.

**Acceptance Criteria**:
- [ ] Both endpoints updated to use `get_current_user_optional()` dependency
- [ ] Logic flow:
  1. Extract user context from token (None if anonymous)
  2. Perform Qdrant vector search (same as before)
  3. Apply score boosting if user context exists (call `apply_score_boosting()`)
  4. Generate LLM response with boosted context
  5. Add metadata to response: `personalized: true/false`, `user_id` (if authenticated)
- [ ] Response metadata includes:
  - `personalized: true` if user authenticated and boosting applied
  - `personalized: false` if anonymous or boosting skipped
  - `user_id: UUID` if authenticated (for logging/analytics)
- [ ] Backward compatibility: Anonymous requests work exactly as before (no breaking changes)

**Test Cases**:
1. Authenticated Python user: Ask "How to create ROS2 node?" → Response includes Python (rclpy) examples first
2. Authenticated C++ user: Ask same question → Response includes C++ (rclcpp) examples first
3. Anonymous user: Ask same question → Response includes balanced Python/C++ examples
4. Metadata verification: Check response → `personalized: true` for authenticated, `false` for anonymous
5. Invalid token: Send malformed token → Treated as anonymous (no error, graceful degradation)
6. Performance: Authenticated request completes in <5s (no degradation vs anonymous)

**Files Modified**:
- `rag-backend/chatbot/app.py` (modify `/chat` and `/stream-chat` routes, ~60 LOC changes)

**Code Template**:
```python
from middleware import get_current_user_optional

@app.post("/stream-chat")
async def stream_chat(
    query: Query,
    user_context: Optional[dict] = Depends(get_current_user_optional)
):
    # Existing Qdrant search logic
    search_results = await qdrant_client.search(...)

    # Apply personalization if authenticated
    if user_context:
        search_results = apply_score_boosting(search_results, user_context)

    # Existing LLM generation logic
    response = await generate_response(search_results, query.question)

    # Add personalization metadata
    metadata = {
        "personalized": user_context is not None,
        "user_id": user_context.get("sub") if user_context else None
    }

    # Stream response with metadata
    async for token in response:
        yield {"token": token, "done": False}

    yield {
        "done": True,
        "response": {
            "answer": full_response,
            "sources": sources,
            **metadata
        }
    }
```

**Reference**: [contracts/api.md - Enhanced Chatbot Endpoints](./contracts/api.md#6-post-stream-chat-enhanced-with-auth)

---

## Phase 3: Frontend Auth UI (Priority: P1)

### T-013: Create Auth Context Provider
**Priority**: P1
**Estimated Effort**: 1 hour
**Assignee**: Frontend Developer
**Dependencies**: None (can start in parallel with backend)

**Description**:
Create React Context for managing authentication state (user, token) across the frontend application.

**Acceptance Criteria**:
- [X] File created: `my-website/src/components/Auth/AuthContext.tsx`
- [ ] Context provides:
  - `user: UserProfile | null` (current authenticated user)
  - `token: string | null` (JWT token)
  - `isAuthenticated: boolean` (convenience flag)
  - `signin: (email, password) => Promise<void>` (calls API, stores token)
  - `signup: (email, password, programming_experience) => Promise<void>`
  - `signout: () => void` (clears token from localStorage)
  - `updateProfile: (programming_experience) => Promise<void>`
- [ ] Token stored in localStorage with key `auth_token`
- [ ] On mount, check localStorage for existing token:
  - If token exists, call GET /auth/me to verify validity
  - If valid, set user state
  - If invalid/expired, clear token from localStorage
- [ ] All API calls include error handling (network errors, 401, etc.)

**Test Cases**:
1. Initial load with no token: `user === null`, `isAuthenticated === false`
2. Initial load with valid token: Calls GET /auth/me, sets user state
3. Initial load with expired token: Clears token, `user === null`
4. Signin success: Calls API, stores token, updates user state
5. Signin failure: Shows error, does not store token
6. Signout: Clears localStorage, sets `user === null`

**Files Created**:
- `my-website/src/components/Auth/AuthContext.tsx` (~120 LOC)

**Code Template**:
```typescript
import React, { createContext, useState, useEffect, useContext } from 'react';
import { signin as signinAPI, signup as signupAPI, getProfile } from '../ChatBot/api';

interface UserProfile {
  id: string;
  email: string;
  programming_experience: string;
  created_at: string;
  updated_at: string;
}

interface AuthContextType {
  user: UserProfile | null;
  token: string | null;
  isAuthenticated: boolean;
  signin: (email: string, password: string) => Promise<void>;
  signup: (email: string, password: string, programming_experience: string) => Promise<void>;
  signout: () => void;
  updateProfile: (programming_experience: string) => Promise<void>;
}

const AuthContext = createContext<AuthContextType | undefined>(undefined);

export const AuthProvider: React.FC<{ children: React.ReactNode }> = ({ children }) => {
  const [user, setUser] = useState<UserProfile | null>(null);
  const [token, setToken] = useState<string | null>(null);

  useEffect(() => {
    const storedToken = localStorage.getItem('auth_token');
    if (storedToken) {
      getProfile(storedToken)
        .then(profile => {
          setUser(profile);
          setToken(storedToken);
        })
        .catch(() => {
          localStorage.removeItem('auth_token');
        });
    }
  }, []);

  const signin = async (email: string, password: string) => {
    const response = await signinAPI(email, password);
    localStorage.setItem('auth_token', response.access_token);
    setToken(response.access_token);
    setUser(response.user);
  };

  const signup = async (email: string, password: string, programming_experience: string) => {
    const response = await signupAPI(email, password, programming_experience);
    localStorage.setItem('auth_token', response.access_token);
    setToken(response.access_token);
    setUser(response.user);
  };

  const signout = () => {
    localStorage.removeItem('auth_token');
    setToken(null);
    setUser(null);
  };

  const updateProfile = async (programming_experience: string) => {
    if (!token) throw new Error('Not authenticated');
    const updatedUser = await updateProfileAPI(token, programming_experience);
    setUser(updatedUser);
  };

  return (
    <AuthContext.Provider value={{
      user,
      token,
      isAuthenticated: !!user,
      signin,
      signup,
      signout,
      updateProfile
    }}>
      {children}
    </AuthContext.Provider>
  );
};

export const useAuth = () => {
  const context = useContext(AuthContext);
  if (!context) throw new Error('useAuth must be used within AuthProvider');
  return context;
};
```

**Reference**: [plan.md - Frontend Integration](./plan.md#frontend-integration)

---

### T-014: Create Signup Modal Component
**Priority**: P1
**Estimated Effort**: 2 hours
**Assignee**: Frontend Developer
**Dependencies**: T-013

**Description**:
Create modal component for user signup with email, password, and programming experience fields.

**Acceptance Criteria**:
- [X] File created: `my-website/src/components/Auth/SignupModal.tsx`
- [ ] Modal UI includes:
  - Email input (type="email", required)
  - Password input (type="password", required, with visibility toggle)
  - Programming experience dropdown (options: None, Python, C++, Both Python and C++)
  - Signup button (disabled while submitting)
  - Link to switch to signin modal ("Already have an account? Sign in")
  - Close button (X in top-right corner)
- [ ] Form validation:
  - Email: Valid format (browser validation)
  - Password: Min 8 chars, shows strength indicator (weak/medium/strong)
  - Programming experience: Required selection
- [ ] Submits signup request via `useAuth().signup()`
- [ ] Error handling:
  - 409: Show "Email already registered" message
  - 422: Show validation errors from API
  - Network error: Show "Connection failed, please try again"
- [ ] Success: Close modal, show welcome message

**Test Cases**:
1. Valid signup: Fill form, click Signup → Modal closes, user authenticated
2. Duplicate email: Signup with existing email → Shows "Email already registered"
3. Weak password: Enter "weak" → Shows strength indicator "Weak", API returns 422
4. Invalid email: Enter "notanemail" → Browser validation prevents submit
5. Switch to signin: Click "Already have an account?" → Opens signin modal
6. Close modal: Click X → Modal closes, form resets

**Files Created**:
- `my-website/src/components/Auth/SignupModal.tsx` (~200 LOC)
- `my-website/src/components/Auth/styles.module.css` (~60 LOC)

**Code Template** (React with TypeScript):
```typescript
import React, { useState } from 'react';
import { useAuth } from './AuthContext';
import styles from './styles.module.css';

interface SignupModalProps {
  isOpen: boolean;
  onClose: () => void;
  onSwitchToSignin: () => void;
}

export const SignupModal: React.FC<SignupModalProps> = ({ isOpen, onClose, onSwitchToSignin }) => {
  const { signup } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [programmingExperience, setProgrammingExperience] = useState('None');
  const [showPassword, setShowPassword] = useState(false);
  const [error, setError] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsSubmitting(true);

    try {
      await signup(email, password, programmingExperience);
      onClose();
    } catch (err: any) {
      if (err.response?.status === 409) {
        setError('Email already registered');
      } else if (err.response?.status === 422) {
        setError(err.response.data.detail);
      } else {
        setError('Connection failed, please try again');
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  if (!isOpen) return null;

  return (
    <div className={styles.modalOverlay} onClick={onClose}>
      <div className={styles.modalContent} onClick={(e) => e.stopPropagation()}>
        <button className={styles.closeButton} onClick={onClose}>×</button>
        <h2>Sign Up</h2>
        {error && <div className={styles.error}>{error}</div>}
        <form onSubmit={handleSubmit}>
          <input
            type="email"
            placeholder="Email"
            value={email}
            onChange={(e) => setEmail(e.target.value)}
            required
          />
          <div className={styles.passwordField}>
            <input
              type={showPassword ? 'text' : 'password'}
              placeholder="Password (min 8 characters)"
              value={password}
              onChange={(e) => setPassword(e.target.value)}
              required
              minLength={8}
            />
            <button type="button" onClick={() => setShowPassword(!showPassword)}>
              {showPassword ? '👁️' : '👁️‍🗨️'}
            </button>
          </div>
          <select
            value={programmingExperience}
            onChange={(e) => setProgrammingExperience(e.target.value)}
            required
          >
            <option value="None">None (I'm a beginner)</option>
            <option value="Python">Python</option>
            <option value="C++">C++</option>
            <option value="Both Python and C++">Both Python and C++</option>
          </select>
          <button type="submit" disabled={isSubmitting}>
            {isSubmitting ? 'Signing up...' : 'Sign Up'}
          </button>
        </form>
        <p>
          Already have an account?{' '}
          <button onClick={onSwitchToSignin} className={styles.linkButton}>
            Sign in
          </button>
        </p>
      </div>
    </div>
  );
};
```

**Reference**: [spec.md - User Story 1](./spec.md#user-story-1---basic-signup--signin-priority-p1)

---

### T-015: Create Signin Modal Component
**Priority**: P1
**Estimated Effort**: 1 hour
**Assignee**: Frontend Developer
**Dependencies**: T-013

**Description**:
Create modal component for user signin with email and password.

**Acceptance Criteria**:
- [X] File created: `my-website/src/components/Auth/SigninModal.tsx`
- [ ] Modal UI includes:
  - Email input (type="email", required)
  - Password input (type="password", required, with visibility toggle)
  - Signin button (disabled while submitting)
  - Link to switch to signup modal ("Don't have an account? Sign up")
  - Close button (X in top-right corner)
- [ ] Submits signin request via `useAuth().signin()`
- [ ] Error handling:
  - 401: Show "Invalid email or password"
  - Network error: Show "Connection failed, please try again"
- [ ] Success: Close modal, user authenticated

**Test Cases**:
1. Valid signin: Enter correct credentials → Modal closes, user authenticated
2. Wrong password: Enter incorrect password → Shows "Invalid email or password"
3. Non-existent email: Enter unregistered email → Shows "Invalid email or password"
4. Switch to signup: Click "Don't have an account?" → Opens signup modal
5. Close modal: Click X → Modal closes, form resets
6. Remember credentials: Browser autofill works

**Files Created**:
- `my-website/src/components/Auth/SigninModal.tsx` (~150 LOC)

**Code Template** (similar to SignupModal, simpler form):
```typescript
export const SigninModal: React.FC<SigninModalProps> = ({ isOpen, onClose, onSwitchToSignup }) => {
  const { signin } = useAuth();
  const [email, setEmail] = useState('');
  const [password, setPassword] = useState('');
  const [error, setError] = useState('');
  const [isSubmitting, setIsSubmitting] = useState(false);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();
    setError('');
    setIsSubmitting(true);

    try {
      await signin(email, password);
      onClose();
    } catch (err: any) {
      if (err.response?.status === 401) {
        setError('Invalid email or password');
      } else {
        setError('Connection failed, please try again');
      }
    } finally {
      setIsSubmitting(false);
    }
  };

  // ... rest similar to SignupModal but simpler form
};
```

**Reference**: [spec.md - User Story 1](./spec.md#user-story-1---basic-signup--signin-priority-p1)

---

### T-016: Create Profile Settings Modal Component
**Priority**: P2
**Estimated Effort**: 1 hour
**Assignee**: Frontend Developer
**Dependencies**: T-013

**Description**:
Create modal component for viewing and updating user profile (programming experience).

**Acceptance Criteria**:
- [X] File created: `my-website/src/components/Auth/ProfileSettingsModal.tsx`
- [ ] Modal UI includes:
  - Email display (read-only)
  - Programming experience dropdown (editable)
  - Save button (disabled while submitting)
  - Close button
- [ ] Pre-fills current user's programming experience
- [ ] Submits update via `useAuth().updateProfile()`
- [ ] Shows success message after update
- [ ] Error handling for network failures

**Test Cases**:
1. Open modal: Pre-fills current programming experience
2. Update: Change to "C++", click Save → Success message, user state updated
3. Verify persistence: Close modal, reopen → Shows "C++"
4. Cancel: Close modal without saving → No changes applied
5. Network error: API fails → Shows error message

**Files Created**:
- `my-website/src/components/Auth/ProfileSettingsModal.tsx` (~120 LOC)

**Reference**: [spec.md - User Story 4](./spec.md#user-story-4---profile-management-and-preference-updates-priority-p2)

---

### T-017: Create Header Auth Button Component
**Priority**: P1
**Estimated Effort**: 1.5 hours
**Assignee**: Frontend Developer
**Dependencies**: T-013, T-014, T-015, T-016

**Description**:
Create header button that shows "Sign In / Sign Up" for anonymous users or profile dropdown for authenticated users.

**Acceptance Criteria**:
- [X] File created: `my-website/src/components/Auth/AuthButton.tsx`
- [ ] Conditional rendering:
  - Anonymous: Shows "Sign In / Sign Up" button
  - Authenticated: Shows user email + dropdown icon
- [ ] Clicking anonymous button opens signin modal (with link to signup)
- [ ] Clicking authenticated button opens dropdown menu with:
  - "Profile Settings" (opens profile modal)
  - "Sign Out" (calls `useAuth().signout()`)
- [ ] Integrates with Docusaurus header theme
- [ ] Responsive design (mobile-friendly)

**Test Cases**:
1. Anonymous user: Shows "Sign In / Sign Up" button
2. Click anonymous button: Opens signin modal
3. Authenticated user: Shows email (e.g., "user@example.com")
4. Click email: Opens dropdown with "Profile Settings" and "Sign Out"
5. Click "Sign Out": User signed out, button shows "Sign In / Sign Up"
6. Mobile view: Button adapts to small screen (icon only or condensed text)

**Files Created**:
- `my-website/src/components/Auth/AuthButton.tsx` (~100 LOC)

**Integration Point**:
- `my-website/src/theme/Layout/index.tsx` (modified to add AuthButton to header)

**Reference**: [spec.md - Frontend Integration](./spec.md#frontend-integration)

---

### T-018: Update API Client with Auth Methods
**Priority**: P1
**Estimated Effort**: 1 hour
**Assignee**: Frontend Developer
**Dependencies**: None (can start early)

**Description**:
Add authentication methods to the frontend API client and include Authorization header in chatbot requests.

**Acceptance Criteria**:
- [X] File modified: `my-website/src/components/ChatBot/api.ts`
- [ ] New methods added:
  - `signup(email, password, programming_experience): Promise<AuthResponse>`
  - `signin(email, password): Promise<AuthResponse>`
  - `getProfile(token): Promise<UserProfile>`
  - `updateProfile(token, programming_experience): Promise<UserProfile>`
- [ ] Existing methods modified:
  - `sendChatMessage()` and `streamChatMessage()` accept optional `token` parameter
  - If token provided, include `Authorization: Bearer <token>` header
- [ ] Error handling for auth-specific errors (401, 409, 422)

**Test Cases**:
1. Signup API call: `signup("test@example.com", "Pass123!", "Python")` → Returns token + user
2. Signin API call: `signin("test@example.com", "Pass123!")` → Returns token + user
3. Get profile: `getProfile(token)` → Returns user profile
4. Chatbot with auth: `streamChatMessage("question", "", token)` → Includes Authorization header
5. Chatbot without auth: `streamChatMessage("question", "")` → No Authorization header

**Files Modified**:
- `my-website/src/components/ChatBot/api.ts` (+100 LOC)

**Code Template**:
```typescript
export interface AuthResponse {
  access_token: string;
  token_type: string;
  user: UserProfile;
}

export interface UserProfile {
  id: string;
  email: string;
  programming_experience: string;
  created_at: string;
  updated_at: string;
}

export async function signup(
  email: string,
  password: string,
  programming_experience: string
): Promise<AuthResponse> {
  const response = await fetch(`${API_BASE_URL}/auth/signup`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password, programming_experience })
  });

  if (!response.ok) {
    throw new Error(`Signup failed: ${response.status}`);
  }

  return response.json();
}

export async function signin(email: string, password: string): Promise<AuthResponse> {
  const response = await fetch(`${API_BASE_URL}/auth/signin`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ email, password })
  });

  if (!response.ok) {
    throw new Error(`Signin failed: ${response.status}`);
  }

  return response.json();
}

export async function getProfile(token: string): Promise<UserProfile> {
  const response = await fetch(`${API_BASE_URL}/auth/me`, {
    headers: { 'Authorization': `Bearer ${token}` }
  });

  if (!response.ok) {
    throw new Error(`Get profile failed: ${response.status}`);
  }

  return response.json();
}

// Modify existing streamChatMessage to include token
export async function* streamChatMessage(
  question: string,
  selectedText: string,
  token?: string
): AsyncGenerator<StreamChunk> {
  const headers: Record<string, string> = {
    'Content-Type': 'application/json'
  };

  if (token) {
    headers['Authorization'] = `Bearer ${token}`;
  }

  const response = await fetch(`${API_BASE_URL}/stream-chat`, {
    method: 'POST',
    headers,
    body: JSON.stringify({ question, selected_text: selectedText })
  });

  // ... rest of streaming logic
}
```

**Reference**: [contracts/api.md - Authentication Endpoints](./contracts/api.md#authentication-endpoints)

---

## Phase 4: Testing & Deployment (Priority: P0 - Critical Path)

### T-019: Write Unit Tests for Backend
**Priority**: P0 (Blocking)
**Estimated Effort**: 3 hours
**Assignee**: Backend Developer
**Dependencies**: T-001 through T-012

**Description**:
Write comprehensive unit tests for authentication and personalization logic.

**Acceptance Criteria**:
- [ ] File created: `rag-backend/chatbot/tests/test_auth.py`
- [ ] Tests for password hashing:
  - Test `hash_password()` produces valid bcrypt hash
  - Test `verify_password()` with correct password → True
  - Test `verify_password()` with wrong password → False
  - Test `validate_password_strength()` with various passwords
- [ ] Tests for JWT tokens:
  - Test `create_access_token()` produces valid JWT
  - Test `decode_access_token()` with valid token → Returns payload
  - Test `decode_access_token()` with expired token → Raises ExpiredSignatureError
  - Test `decode_access_token()` with invalid signature → Raises InvalidTokenError
- [ ] Tests for score boosting:
  - Test Python user boost → Python results prioritized
  - Test C++ user boost → C++ results prioritized
  - Test None user boost → Conceptual content prioritized
  - Test Both user boost → Balanced results
  - Test missing metadata → Graceful handling
- [ ] All tests pass: `pytest tests/test_auth.py -v`
- [ ] Test coverage > 80% for auth modules

**Test Cases** (examples):
```python
def test_password_hashing():
    password = "TestPass123!"
    hashed = hash_password(password)
    assert hashed.startswith("$2b$10$")
    assert verify_password(password, hashed) == True
    assert verify_password("WrongPass", hashed) == False

def test_jwt_token_generation():
    token = create_access_token(uuid4(), "test@example.com", "Python")
    payload = decode_access_token(token)
    assert payload["email"] == "test@example.com"
    assert payload["programming_experience"] == "Python"

def test_score_boosting_python_user():
    results = [
        {"score": 0.9, "payload": {"text": "Python rclpy node example"}},
        {"score": 0.85, "payload": {"text": "C++ rclcpp node example"}}
    ]
    user_context = {"programming_experience": "Python"}
    boosted = apply_score_boosting(results, user_context)
    assert boosted[0]["payload"]["text"].startswith("Python")  # Python result first
```

**Files Created**:
- `rag-backend/chatbot/tests/test_auth.py` (~200 LOC)
- `rag-backend/chatbot/tests/__init__.py`

**Reference**: [plan.md - Definition of Done](./plan.md#definition-of-done-dod)

---

### T-020: Write Integration Tests for Auth Flow
**Priority**: P0 (Blocking)
**Estimated Effort**: 2 hours
**Assignee**: Backend Developer
**Dependencies**: T-019

**Description**:
Write end-to-end integration tests for complete authentication and personalization flows.

**Acceptance Criteria**:
- [ ] File created: `rag-backend/chatbot/tests/test_integration.py`
- [ ] Test scenarios:
  - **Signup → Signin → Authenticated Chat**:
    1. POST /auth/signup with valid data → 201 + token
    2. POST /auth/signin with same credentials → 200 + token
    3. POST /stream-chat with Authorization header → Personalized response
  - **Profile Update → Personalization Changes**:
    1. Signup with "Python" experience
    2. Ask chatbot question → Python examples prioritized
    3. PUT /auth/me to "C++" experience
    4. Ask same question → C++ examples prioritized
  - **Anonymous → Signup → Personalized**:
    1. POST /stream-chat without auth → Generic response (metadata: personalized=false)
    2. POST /auth/signup → 201 + token
    3. POST /stream-chat with auth → Personalized response (metadata: personalized=true)
  - **Expired Token → 401**:
    1. Create token with exp in past
    2. GET /auth/me with expired token → 401
- [ ] All tests pass: `pytest tests/test_integration.py -v`
- [ ] Tests use test database (not production Neon DB)

**Test Cases** (example):
```python
@pytest.mark.integration
def test_signup_signin_authenticated_chat(test_client, test_db):
    # Signup
    signup_response = test_client.post("/auth/signup", json={
        "email": "test@example.com",
        "password": "TestPass123!",
        "programming_experience": "Python"
    })
    assert signup_response.status_code == 201
    token = signup_response.json()["access_token"]

    # Signin
    signin_response = test_client.post("/auth/signin", json={
        "email": "test@example.com",
        "password": "TestPass123!"
    })
    assert signin_response.status_code == 200

    # Authenticated chat
    chat_response = test_client.post("/stream-chat",
        json={"question": "How to create ROS2 node?"},
        headers={"Authorization": f"Bearer {token}"}
    )
    # Parse streaming response
    response_data = parse_stream(chat_response)
    assert response_data["personalized"] == True
    assert "python" in response_data["answer"].lower()
```

**Files Created**:
- `rag-backend/chatbot/tests/test_integration.py` (~150 LOC)

**Reference**: [plan.md - Integration Tests](./plan.md#definition-of-done-dod)

---

### T-021: Set Up Production Deployment
**Priority**: P0 (Blocking)
**Estimated Effort**: 2 hours
**Assignee**: DevOps / Backend Developer
**Dependencies**: T-001 through T-020

**Description**:
Configure environment variables in Hugging Face Spaces and deploy backend with auth features.

**Acceptance Criteria**:
- [ ] Neon DB connection string added to HF Spaces secrets: `NEON_DATABASE_URL`
- [ ] JWT secret added to HF Spaces secrets: `JWT_SECRET_KEY` (generated with `openssl rand -hex 32`)
- [ ] Existing secrets verified: `COHERE_API_KEY`, `QDRANT_URL`, `QDRANT_API_KEY`
- [ ] `requirements.txt` updated with auth dependencies (from T-001)
- [ ] Docker image built with new dependencies
- [ ] Alembic migrations run on production DB: `alembic upgrade head`
- [ ] Backend deployed to HF Spaces (push Docker image)
- [ ] Health check passes: `curl https://rameesha12123214-hackathone.hf.space/health` → 200 OK
- [ ] Verify database connection: Check logs for "Connected to Neon DB" message

**Test Cases**:
1. Verify secrets: Check HF Spaces settings → All secrets present
2. Build Docker image: `docker build -t backend .` → Success
3. Test migrations locally: `alembic upgrade head` → No errors
4. Deploy: Push image to HF Spaces → Deployment succeeds
5. Health check: `curl /health` → Returns 200
6. Database connection: Check logs → No connection errors

**Deployment Checklist**:
- [ ] Update `rag-backend/chatbot/requirements.txt` with all dependencies
- [ ] Update `rag-backend/chatbot/Dockerfile` if needed (should auto-install requirements.txt)
- [ ] Set environment variables in HF Spaces secrets (web UI)
- [ ] Run migrations: Connect to Neon DB and run `alembic upgrade head`
- [ ] Build and push Docker image to HF Spaces
- [ ] Monitor deployment logs for errors
- [ ] Test health endpoint after deployment

**Reference**: [plan.md - Deployment Strategy](./plan.md#deployment-and-rollback-strategies)

---

### T-022: Production Testing and Smoke Tests
**Priority**: P0 (Blocking)
**Estimated Effort**: 1 hour
**Assignee**: Full Stack Developer
**Dependencies**: T-021

**Description**:
Perform end-to-end smoke tests in production environment to verify all features work.

**Acceptance Criteria**:
- [ ] Test signup in production:
  - Open production URL: https://humanoid-robotics-textbook-psi.vercel.app
  - Click "Sign Up", create test account → Success
- [ ] Test signin:
  - Sign out, sign in with test account → Success
- [ ] Test personalized chatbot:
  - Ask "How to create ROS2 node?" while signed in → Response includes Python examples first (if signed up with Python)
- [ ] Test anonymous chatbot:
  - Sign out, ask same question → Response is balanced/generic
- [ ] Test profile update:
  - Open profile settings, change programming experience to "C++" → Success
  - Ask same question again → Response now prioritizes C++ examples
- [ ] Verify metadata:
  - Check chatbot response metadata → `personalized: true` when authenticated, `false` when anonymous
- [ ] Monitor logs for errors (first 10 minutes after deployment)

**Test Cases** (manual):
1. **Signup Flow**:
   - Navigate to homepage
   - Click "Sign In / Sign Up"
   - Click "Sign up" link
   - Fill email: `test-prod@example.com`, password: `TestProd123!`, experience: `Python`
   - Submit → Account created, modal closes, header shows email

2. **Signin Flow**:
   - Click profile dropdown, click "Sign Out"
   - Click "Sign In / Sign Up"
   - Enter credentials → Signed in successfully

3. **Personalized Chat**:
   - Click chatbot button
   - Ask: "How do I create a ROS2 node?"
   - Verify response includes Python (rclpy) code examples prominently

4. **Anonymous Chat**:
   - Sign out
   - Ask same question → Response includes both Python and C++ examples (balanced)

5. **Profile Update**:
   - Sign in
   - Click profile dropdown → "Profile Settings"
   - Change experience to "C++"
   - Save and close
   - Ask same question → Response now prioritizes C++ (rclcpp) examples

6. **Error Handling**:
   - Try signing up with existing email → Shows "Email already registered"
   - Try signing in with wrong password → Shows "Invalid email or password"

**Success Criteria**:
- All manual tests pass without errors
- No console errors in browser dev tools
- No 500 errors in backend logs
- Chatbot response time < 5 seconds
- Authentication flows complete within 2 seconds

**Reference**: [plan.md - Smoke Test](./plan.md#deployment-and-rollback-strategies)

---

## Optional Tasks (Post-MVP Enhancements)

### T-023: Add Password Strength Indicator (Frontend)
**Priority**: P3 (Nice to have)
**Estimated Effort**: 30 minutes
**Dependencies**: T-014

**Description**:
Add visual password strength indicator in signup modal (Weak/Medium/Strong).

**Acceptance Criteria**:
- [ ] Indicator updates as user types password
- [ ] Shows color-coded strength: Red (weak), Yellow (medium), Green (strong)
- [ ] Strength calculated based on length, character variety, common patterns

**Reference**: [spec.md - User Story 1](./spec.md#user-story-1---basic-signup--signin-priority-p1)

---

### T-024: Add Anonymous User Banner in Chatbot
**Priority**: P2
**Estimated Effort**: 30 minutes
**Dependencies**: T-013, T-017

**Description**:
Display informational banner in chatbot for anonymous users promoting signup.

**Acceptance Criteria**:
- [ ] Banner appears at top of chatbot for anonymous users
- [ ] Message: "Sign up for personalized responses based on your programming experience"
- [ ] Clickable: Opens signup modal
- [ ] Dismissible: X button to close (remembered in localStorage)
- [ ] Not shown for authenticated users

**Reference**: [spec.md - User Story 3](./spec.md#user-story-3---anonymous-access-with-limited-personalization-priority-p1), [ADR-004](../adr/004-optional-authentication-model.md)

---

### T-025: Add Signout Endpoint (POST /auth/signout)
**Priority**: P3 (Optional)
**Estimated Effort**: 15 minutes
**Dependencies**: T-010

**Description**:
Create POST /auth/signout endpoint for consistency (mainly client-side operation).

**Acceptance Criteria**:
- [ ] Route: `POST /auth/signout` added to auth router
- [ ] Requires authentication
- [ ] Returns `{"message": "Successfully signed out"}`
- [ ] Note: Stateless JWT means server cannot invalidate token; client must delete from localStorage

**Reference**: [contracts/api.md - POST /auth/signout](./contracts/api.md#5-post-authsignout)

---

## Summary

**Total Tasks**: 22 core tasks + 3 optional
**Estimated Total Effort**: ~26 hours for core MVP
**Critical Path**: T-001 → T-002 → T-003 → T-004 → T-005 → T-006 → T-007 → T-010 → T-011 → T-012 → T-019 → T-020 → T-021 → T-022

**Parallel Work Opportunities**:
- Frontend tasks (T-013 through T-018) can start in parallel with backend Phase 1
- T-004 and T-005 can be developed in parallel (both depend only on T-001)
- T-014 and T-015 can be developed in parallel (both depend only on T-013)

**Key Milestones**:
1. **Backend Foundation Complete**: T-001 through T-010 done (~10 hours)
2. **Personalization Working**: T-011 and T-012 done (+3 hours)
3. **Frontend UI Complete**: T-013 through T-018 done (+7 hours)
4. **Testing Complete**: T-019 and T-020 done (+5 hours)
5. **Production Deployed**: T-021 and T-022 done (+3 hours)

**Next Step**: Run `/sp.implement` to begin implementation, or manually start with Task T-001.
