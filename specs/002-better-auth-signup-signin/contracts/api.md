# API Contracts: Better-Auth Signup & Signin

## Base URL

- **Development**: `http://localhost:8001`
- **Production**: `https://rameesha12123214-hackathone.hf.space`

---

## Authentication Endpoints

### 1. POST /auth/signup

Create a new user account with email, password, and programming experience.

**Request**:
```http
POST /auth/signup HTTP/1.1
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "SecureP@ss123",
  "programming_experience": "Python"
}
```

**Success Response (201 Created)**:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI1NTBlODQwMC1lMjliLTQxZDQtYTcxNi00NDY2NTU0NDAwMDAiLCJlbWFpbCI6InVzZXJAZXhhbXBsZS5jb20iLCJwcm9ncmFtbWluZ19leHBlcmllbmNlIjoiUHl0aG9uIiwiZXhwIjoxNzAzNzIxNjAwLCJpYXQiOjE3MDMxMTY4MDB9.xyz",
  "token_type": "bearer",
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com",
    "programming_experience": "Python",
    "created_at": "2025-12-27T10:30:00Z",
    "updated_at": "2025-12-27T10:30:00Z"
  }
}
```

**Error Responses**:

- **409 Conflict** (Email already exists):
```json
{
  "detail": "Email already registered"
}
```

- **422 Unprocessable Entity** (Validation error):
```json
{
  "detail": [
    {
      "loc": ["body", "password"],
      "msg": "Password must be at least 8 characters and contain uppercase, lowercase, and number",
      "type": "value_error"
    }
  ]
}
```

- **422 Unprocessable Entity** (Invalid programming experience):
```json
{
  "detail": [
    {
      "loc": ["body", "programming_experience"],
      "msg": "Input should be 'None', 'Python', 'C++' or 'Both Python and C++'",
      "type": "literal_error"
    }
  ]
}
```

---

### 2. POST /auth/signin

Authenticate existing user and receive session token.

**Request**:
```http
POST /auth/signin HTTP/1.1
Content-Type: application/json

{
  "email": "user@example.com",
  "password": "SecureP@ss123"
}
```

**Success Response (200 OK)**:
```json
{
  "access_token": "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOiI1NTBlODQwMC1lMjliLTQxZDQtYTcxNi00NDY2NTU0NDAwMDAiLCJlbWFpbCI6InVzZXJAZXhhbXBsZS5jb20iLCJwcm9ncmFtbWluZ19leHBlcmllbmNlIjoiUHl0aG9uIiwiZXhwIjoxNzAzNzIxNjAwLCJpYXQiOjE3MDMxMTY4MDB9.xyz",
  "token_type": "bearer",
  "user": {
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "user@example.com",
    "programming_experience": "Python",
    "created_at": "2025-12-27T10:30:00Z",
    "updated_at": "2025-12-27T15:45:00Z"
  }
}
```

**Error Responses**:

- **401 Unauthorized** (Invalid credentials):
```json
{
  "detail": "Invalid email or password"
}
```

- **422 Unprocessable Entity** (Invalid email format):
```json
{
  "detail": [
    {
      "loc": ["body", "email"],
      "msg": "value is not a valid email address",
      "type": "value_error.email"
    }
  ]
}
```

---

### 3. GET /auth/me

Retrieve current authenticated user's profile.

**Request**:
```http
GET /auth/me HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Success Response (200 OK)**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "programming_experience": "Python",
  "created_at": "2025-12-27T10:30:00Z",
  "updated_at": "2025-12-27T15:45:00Z"
}
```

**Error Responses**:

- **401 Unauthorized** (Missing token):
```json
{
  "detail": "Not authenticated"
}
```

- **401 Unauthorized** (Invalid token):
```json
{
  "detail": "Invalid authentication token"
}
```

- **401 Unauthorized** (Expired token):
```json
{
  "detail": "Token has expired"
}
```

---

### 4. PUT /auth/me

Update current authenticated user's profile (programming experience).

**Request**:
```http
PUT /auth/me HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "programming_experience": "C++"
}
```

**Success Response (200 OK)**:
```json
{
  "id": "550e8400-e29b-41d4-a716-446655440000",
  "email": "user@example.com",
  "programming_experience": "C++",
  "created_at": "2025-12-27T10:30:00Z",
  "updated_at": "2025-12-27T16:00:00Z"
}
```

**Error Responses**:

- **401 Unauthorized** (Missing/invalid token):
```json
{
  "detail": "Not authenticated"
}
```

- **422 Unprocessable Entity** (Invalid programming experience):
```json
{
  "detail": [
    {
      "loc": ["body", "programming_experience"],
      "msg": "Input should be 'None', 'Python', 'C++' or 'Both Python and C++'",
      "type": "literal_error"
    }
  ]
}
```

---

### 5. POST /auth/signout

Sign out current user (invalidate token on client side).

**Request**:
```http
POST /auth/signout HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
```

**Success Response (200 OK)**:
```json
{
  "message": "Successfully signed out"
}
```

**Error Responses**:

- **401 Unauthorized** (Missing/invalid token):
```json
{
  "detail": "Not authenticated"
}
```

**Note**: Since JWT tokens are stateless, signout is primarily handled client-side by deleting the token from localStorage/cookies. This endpoint is optional but included for consistency and potential future token blacklisting.

---

## Enhanced Chatbot Endpoints (Modified)

### 6. POST /stream-chat (Enhanced with Auth)

Stream chatbot responses with optional personalization based on user authentication.

**Request (Authenticated User)**:
```http
POST /stream-chat HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "question": "How do I create a ROS2 node?",
  "selected_text": "",
  "agent_id": null
}
```

**Request (Anonymous User)**:
```http
POST /stream-chat HTTP/1.1
Content-Type: application/json

{
  "question": "How do I create a ROS2 node?",
  "selected_text": "",
  "agent_id": null
}
```

**Success Response (SSE Stream)**:

For authenticated user with "Python" experience:
```
data: {"token": "To", "done": false}

data: {"token": " create", "done": false}

data: {"token": " a", "done": false}

data: {"token": " ROS2", "done": false}

data: {"token": " node", "done": false}

data: {"token": " in", "done": false}

data: {"token": " Python", "done": false}

data: {"token": ",", "done": false}

data: {"token": " use", "done": false}

data: {"token": " rclpy", "done": false}

... (more tokens)

data: {"done": true, "response": {"answer": "To create a ROS2 node in Python, use rclpy...", "sources": ["https://humanoid-robotics-textbook-psi.vercel.app/docs/module-1-ros2/nodes-topics#python-node"], "personalized": true, "user_id": "550e8400-e29b-41d4-a716-446655440000"}}
```

For anonymous user (no personalization):
```
... (streaming tokens)

data: {"done": true, "response": {"answer": "To create a ROS2 node, you can use either Python (rclpy) or C++ (rclcpp)...", "sources": ["https://humanoid-robotics-textbook-psi.vercel.app/docs/module-1-ros2/nodes-topics"], "personalized": false}}
```

**Error Responses**:

- **500 Internal Server Error** (Qdrant connection failure):
```json
{
  "error": "Failed to connect to vector database"
}
```

- **401 Unauthorized** (Invalid token - but doesn't block anonymous access):
  - If token is provided but invalid, backend falls back to anonymous mode
  - No error response; user receives generic (non-personalized) results

---

### 7. POST /chat (Enhanced with Auth)

Non-streaming chatbot endpoint with optional personalization.

**Request (Authenticated User)**:
```http
POST /chat HTTP/1.1
Authorization: Bearer eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9...
Content-Type: application/json

{
  "question": "Explain ROS2 topics",
  "selected_text": ""
}
```

**Success Response (200 OK)**:

For authenticated user with "C++" experience:
```json
{
  "answer": "ROS2 topics are communication channels that allow nodes to exchange messages. Here's a C++ example using rclcpp...",
  "sources": [
    "https://humanoid-robotics-textbook-psi.vercel.app/docs/module-1-ros2/nodes-topics#cpp-publisher",
    "https://humanoid-robotics-textbook-psi.vercel.app/docs/module-1-ros2/nodes-topics#cpp-subscriber"
  ],
  "personalized": true,
  "user_id": "550e8400-e29b-41d4-a716-446655440000"
}
```

For anonymous user:
```json
{
  "answer": "ROS2 topics are communication channels that allow nodes to exchange messages. You can implement publishers and subscribers in both Python (rclpy) and C++ (rclcpp)...",
  "sources": [
    "https://humanoid-robotics-textbook-psi.vercel.app/docs/module-1-ros2/nodes-topics"
  ],
  "personalized": false
}
```

---

## CORS Configuration

**Allowed Origins**:
- `http://localhost:3000` (development)
- `https://humanoid-robotics-textbook-psi.vercel.app` (production)

**Allowed Methods**:
- `GET`, `POST`, `PUT`, `OPTIONS`

**Allowed Headers**:
- `Content-Type`
- `Authorization`

**Exposed Headers**:
- `Content-Type`

**Credentials**: `true` (allow cookies if using httpOnly cookies for token storage)

**FastAPI CORS Middleware Configuration**:
```python
from fastapi.middleware.cors import CORSMiddleware

app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://localhost:3000",
        "https://humanoid-robotics-textbook-psi.vercel.app"
    ],
    allow_credentials=True,
    allow_methods=["GET", "POST", "PUT", "OPTIONS"],
    allow_headers=["Content-Type", "Authorization"],
)
```

---

## Authentication Flow Diagrams

### Signup Flow

```
Client                        FastAPI Backend                  Neon DB
  |                                 |                              |
  |--POST /auth/signup------------->|                              |
  |  {email, password, prog_exp}    |                              |
  |                                 |                              |
  |                                 |--Check email exists--------->|
  |                                 |<--Email available------------|
  |                                 |                              |
  |                                 |--Hash password (bcrypt)      |
  |                                 |--INSERT user---------------->|
  |                                 |<--User ID returned-----------|
  |                                 |                              |
  |                                 |--Generate JWT token          |
  |                                 |  (7 day expiration)          |
  |                                 |                              |
  |<--201 {token, user}-------------|                              |
  |                                 |                              |
  |--Store token in localStorage    |                              |
```

### Signin Flow

```
Client                        FastAPI Backend                  Neon DB
  |                                 |                              |
  |--POST /auth/signin------------->|                              |
  |  {email, password}              |                              |
  |                                 |                              |
  |                                 |--SELECT user by email------->|
  |                                 |<--User record----------------|
  |                                 |                              |
  |                                 |--Verify password (bcrypt)    |
  |                                 |  [constant-time comparison]  |
  |                                 |                              |
  |                                 |--Generate JWT token          |
  |                                 |  (7 day expiration)          |
  |                                 |                              |
  |<--200 {token, user}-------------|                              |
  |                                 |                              |
  |--Store token in localStorage    |                              |
```

### Personalized Chat Flow

```
Client                   FastAPI Backend              Qdrant DB        Cohere API
  |                            |                          |                 |
  |--POST /stream-chat-------->|                          |                 |
  | Authorization: Bearer xyz  |                          |                 |
  |                            |                          |                 |
  |                            |--Validate JWT token      |                 |
  |                            |--Extract user context    |                 |
  |                            |  {prog_exp: "Python"}    |                 |
  |                            |                          |                 |
  |                            |--Embed query------------>|                 |
  |                            |<--Query embedding--------|                 |
  |                            |                          |                 |
  |                            |--Vector search---------->|                 |
  |                            |  + boost filter          |                 |
  |                            |  (Python chunks * 1.3)   |                 |
  |                            |<--Top 5 chunks-----------|                 |
  |                            |                          |                 |
  |                            |--Generate response------>|                 |
  |                            |  (Python-focused context)|                 |
  |<--SSE stream (tokens)------|<--Stream tokens----------|                 |
  |                            |                          |                 |
  |<--Final response-----------|                          |                 |
  | {personalized: true}       |                          |                 |
```

---

## Error Handling Strategy

**HTTP Status Code Usage**:
- `200 OK`: Successful request
- `201 Created`: Successful resource creation (signup)
- `400 Bad Request`: Malformed request body
- `401 Unauthorized`: Missing, invalid, or expired authentication token
- `403 Forbidden`: User authenticated but lacks permission (future use)
- `404 Not Found`: Resource doesn't exist (future use)
- `409 Conflict`: Duplicate resource (email already registered)
- `422 Unprocessable Entity`: Validation error (invalid data format)
- `500 Internal Server Error`: Unexpected server error

**Error Response Format** (consistent across all endpoints):
```json
{
  "detail": "Human-readable error message"
}
```

**Validation Error Format** (Pydantic default):
```json
{
  "detail": [
    {
      "loc": ["body", "field_name"],
      "msg": "Error message",
      "type": "error_type"
    }
  ]
}
```

---

## Rate Limiting (Future Enhancement)

**Recommended Limits** (not implemented in MVP):
- Signup: 5 requests per IP per hour
- Signin: 10 requests per IP per 15 minutes
- Profile update: 20 requests per user per hour
- Chat: 60 requests per user per minute (anonymous: 20 per IP per minute)

**Implementation Note**: Use middleware like `slowapi` or `fastapi-limiter` for production deployment.
