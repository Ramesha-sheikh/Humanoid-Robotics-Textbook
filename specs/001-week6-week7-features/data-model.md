# Data Model: Educational Content Features

## Entities

### User
Represents a user of the educational platform. This entity is assumed to be managed by an existing authentication system, but key attributes relevant to our features are listed.

*   **id**: Unique identifier for the user (string, primary key)
*   **username**: User's chosen username (string, unique)
*   **email**: User's email address (string, unique)
*   **roles**: List of roles (e.g., "student", "instructor", "admin") (list of strings)

### Course
Represents a high-level educational course, composed of multiple learning modules.

*   **id**: Unique identifier for the course (string, primary key)
*   **title**: Title of the course (string)
*   **description**: Short description of the course (string)
*   **slug**: URL-friendly identifier for the course (string, unique)
*   **modules**: List of `LearningModule` IDs belonging to this course (list of strings)
*   **created_at**: Timestamp when the course was created (datetime)
*   **updated_at**: Timestamp when the course was last updated (datetime)

### LearningModule
Represents a specific learning module or chapter within a course.

*   **id**: Unique identifier for the learning module (string, primary key)
*   **course_id**: ID of the `Course` this module belongs to (string, foreign key to `Course.id`)
*   **title**: Title of the learning module (string)
*   **description**: Short description of the module (string)
*   **slug**: URL-friendly identifier for the module (string, unique within a course)
*   **content_path**: Path to the markdown/MDX content file for this module (string)
*   **type_insight**: Content for Theory Insight (string, e.g., markdown content or reference to content)
*   **hands_on_exercise**: Content for Hands-on Exercise (string, e.g., markdown content or reference to content)
*   **real_world_application**: Content for Real-world Application (string, e.g., markdown content or reference to content)
*   **order**: Order of the module within its course (integer)
*   **created_at**: Timestamp when the module was created (datetime)
*   **updated_at**: Timestamp when the module was last updated (datetime)

### UserProgress
Tracks a user's progress through a specific learning module.

*   **id**: Unique identifier for the progress record (string, primary key)
*   **user_id**: ID of the `User` (string, foreign key to `User.id`)
*   **module_id**: ID of the `LearningModule` (string, foreign key to `LearningModule.id`)
*   **status**: Current status (e.g., "not_started", "in_progress", "completed") (string)
*   **score**: Score for interactive exercises within the module (integer, nullable)
*   **completed_at**: Timestamp when the module was completed (datetime, nullable)
*   **started_at**: Timestamp when the module was first started (datetime)
*   **last_accessed_at**: Timestamp when the module was last accessed (datetime)

## Relationships

*   **User** 1:N **UserProgress**: One user can have progress records for multiple modules.
*   **Course** 1:N **LearningModule**: One course can contain multiple learning modules.
*   **LearningModule** 1:N **UserProgress**: One learning module can have progress records from multiple users.
*   **Course** 1:N **LearningModule**: A `Course` implicitly contains `LearningModule` entities through its `modules` attribute. The `LearningModule.course_id` explicitly links it back to the `Course`.
