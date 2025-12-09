# Research Findings: Docusaurus and FastAPI Integration for Educational Content

## Docusaurus Content Structure Best Practices

**Decision:** Organize educational content in Docusaurus using a hierarchical directory structure with sidebar configuration for navigation.

**Rationale:** This approach aligns with Docusaurus's native capabilities for documentation sites, offering clear logical grouping of content (e.g., `docs/week6/`, `docs/week7/`). Sidebars provide an intuitive navigation experience for users, mimicking a textbook's table of contents. Markdown files are the primary content format, allowing for rich text, code blocks, and embedded components.

**Alternatives Considered:** Flat structure (rejected due to lack of organization for complex educational content), custom navigation systems (rejected for complexity and deviation from Docusaurus's strengths).

**Recommended Directory Structure:**
```
docs/
├── week6/
│   ├── _category_.json  # Defines sidebar label for "Week 6"
│   ├── lesson1.mdx
│   ├── lesson2.mdx
│   └── ...
├── week7/
│   ├── _category_.json  # Defines sidebar label for "Week 7"
│   ├── lessonA.mdx
│   ├── lessonB.mdx
│   └── ...
├── introduction.md
└── ...
```
`.mdx` files are preferred for embedding custom React components to handle dynamic data (e.g., user progress, interactive exercises).

## FastAPI and Docusaurus Integration Patterns

**Decision:** Utilize Client-Side Rendering (CSR) within Docusaurus React components to make API calls to the FastAPI backend for dynamic data. Implement OAuth2 with JWTs for authentication and leverage FastAPI's dependency injection for protected endpoints and RBAC.

**Rationale:** CSR ensures that dynamic, user-specific data (like user progress) is fetched and rendered in real-time in the user's browser, as Docusaurus primarily serves static pages. FastAPI provides robust tools for building secure and scalable APIs, with JWTs offering a standard and efficient way to handle authentication. CORS configuration will be essential to allow communication between the frontend and backend.

**Alternatives Considered:** Build-time data fetching (rejected for user-specific dynamic data, as it would require frequent redeployments), server-side rendering (not native to Docusaurus, would require significant custom setup).

**Key Integration Aspects:**
*   **API Calls from Docusaurus Components:** Custom React components (`src/components/`) will use `fetch` or Axios to make HTTP requests to FastAPI endpoints.
*   **Authentication:** FastAPI will handle user login, issue JWTs upon successful authentication, and validate these tokens for protected resources. JWTs will be stored in `localStorage` (with XSS considerations) or `HttpOnly` cookies (more secure for refresh tokens).
*   **Authorization:** FastAPI will enforce RBAC based on user roles (e.g., student, instructor) extracted from JWTs. The frontend can adapt UI based on these roles.
*   **CORS:** FastAPI will be configured with `CORSMiddleware` to allow requests from the Docusaurus frontend's origin(s).
*   **Example Workflow:** User logs in via a Docusaurus component -> FastAPI returns JWT -> Docusaurus component stores JWT -> Subsequent API calls from Docusaurus components include JWT in `Authorization` header -> FastAPI validates JWT and serves dynamic data.
