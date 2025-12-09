# API Contracts: Educational Content Backend (FastAPI)

This document outlines the RESTful API endpoints for managing educational content (courses, learning modules) and tracking user progress. The API will be implemented using FastAPI and secured with OAuth2/JWT for authentication.

## Base URL
`/api/v1`

## Authentication
All endpoints requiring user-specific data will be protected with JWT authentication. Clients must send a `Bearer` token in the `Authorization` header.

### `POST /api/v1/token`
**Description**: Authenticates a user and issues a JWT.
**Request Body** (`application/json`):
```json
{
  "username": "string",
  "password": "string"
}
```
**Response** (`application/json`, `200 OK`):
```json
{
  "access_token": "string",
  "token_type": "bearer"
}
```
**Error Responses**:
*   `401 Unauthorized`: Invalid credentials.

## Endpoints

### Courses

#### `GET /api/v1/courses`
**Description**: Retrieves a list of all available courses.
**Response** (`application/json`, `200 OK`):
```json
[
  {
    "id": "string",
    "title": "string",
    "description": "string",
    "slug": "string"
  }
]
```

#### `GET /api/v1/courses/{course_id}`
**Description**: Retrieves details for a specific course, including its modules.
**Path Parameters**:
*   `course_id`: The ID of the course (string).
**Response** (`application/json`, `200 OK`):
```json
{
  "id": "string",
  "title": "string",
  "description": "string",
  "slug": "string",
  "modules": [
    {
      "id": "string",
      "title": "string",
      "slug": "string",
      "order": 0
    }
  ]
}
```
**Error Responses**:
*   `404 Not Found`: Course not found.

### Learning Modules

#### `GET /api/v1/modules/{module_id}`
**Description**: Retrieves details for a specific learning module.
**Path Parameters**:
*   `module_id`: The ID of the learning module (string).
**Response** (`application/json`, `200 OK`):
```json
{
  "id": "string",
  "course_id": "string",
  "title": "string",
  "description": "string",
  "slug": "string",
  "content_path": "string",
  "order": 0,
  "type_insight": "string",
  "hands_on_exercise": "string",
  "real_world_application": "string"
}
```
**Error Responses**:
*   `404 Not Found`: Module not found.

### User Progress (Protected Endpoints)

#### `GET /api/v1/users/{user_id}/progress`
**Description**: Retrieves all progress records for a specific user. (Requires authentication)
**Path Parameters**:
*   `user_id`: The ID of the user (string).
**Response** (`application/json`, `200 OK`):
```json
[
  {
    "id": "string",
    "user_id": "string",
    "module_id": "string",
    "status": "completed",
    "score": 90,
    "completed_at": "2025-12-07T10:00:00Z",
    "started_at": "2025-12-07T09:00:00Z",
    "last_accessed_at": "2025-12-07T10:00:00Z"
  }
]
```
**Error Responses**:
*   `401 Unauthorized`: Missing or invalid token.
*   `403 Forbidden`: User not authorized to view this progress (e.g., trying to view another user's progress without instructor/admin role).
*   `404 Not Found`: User or progress records not found.

#### `GET /api/v1/users/{user_id}/modules/{module_id}/progress`
**Description**: Retrieves a specific user's progress for a given module. (Requires authentication)
**Path Parameters**:
*   `user_id`: The ID of the user (string).
*   `module_id`: The ID of the learning module (string).
**Response** (`application/json`, `200 OK`):
```json
{
  "id": "string",
  "user_id": "string",
  "module_id": "string",
  "status": "in_progress",
  "score": null,
  "completed_at": null,
  "started_at": "2025-12-07T09:00:00Z",
  "last_accessed_at": "2025-12-07T10:00:00Z"
}
```
**Error Responses**:
*   `401 Unauthorized`: Missing or invalid token.
*   `403 Forbidden`: User not authorized.
*   `404 Not Found`: Progress record not found for the user and module.

#### `PUT /api/v1/users/{user_id}/modules/{module_id}/progress`
**Description**: Updates a user's progress for a specific module. (Requires authentication)
**Path Parameters**:
*   `user_id`: The ID of the user (string).
*   `module_id`: The ID of the learning module (string).
**Request Body** (`application/json`):
```json
{
  "status": "completed",
  "score": 95,
  "completed_at": "2025-12-07T11:00:00Z",
  "started_at": "2025-12-07T09:00:00Z",
  "last_accessed_at": "2025-12-07T11:00:00Z"
}
```
**Response** (`application/json`, `200 OK`):
```json
{
  "id": "string",
  "user_id": "string",
  "module_id": "string",
  "status": "completed",
  "score": 95,
  "completed_at": "2025-12-07T11:00:00Z",
  "started_at": "2025-12-07T09:00:00Z",
  "last_accessed_at": "2025-12-07T11:00:00Z"
}
```
**Error Responses**:
*   `401 Unauthorized`: Missing or invalid token.
*   `403 Forbidden`: User not authorized.
*   `404 Not Found`: Progress record not found.
*   `422 Unprocessable Entity`: Invalid input data.
