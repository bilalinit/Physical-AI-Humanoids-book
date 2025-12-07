# Data Model: Docusaurus Book for Physical AI & Humanoid Robotics

## Entities

### User
- **id**: string (UUID) - Unique identifier for each user
- **email**: string - User's email address
- **name**: string - User's full name
- **profile**: object - User background information
  - education: string (e.g., "undergraduate", "graduate", "professional")
  - software_experience: string (e.g., "beginner", "intermediate", "advanced")
  - hardware_experience: string (e.g., "none", "basic", "experienced")
  - robotics_background: string (e.g., "none", "student", "researcher", "engineer")
- **created_at**: datetime - Account creation timestamp
- **updated_at**: datetime - Last profile update timestamp

### DocumentationChapter
- **id**: string (UUID) - Unique identifier for each chapter
- **title**: string - Chapter title
- **slug**: string - URL-friendly identifier
- **part**: string - Part identifier (e.g., "infrastructure", "ros-nervous-system")
- **chapter_number**: integer - Sequential chapter number (1-12)
- **content**: string - Markdown content of the chapter
- **prerequisites**: array of strings - List of prerequisite chapters/sections
- **learning_objectives**: array of strings - Learning objectives for the chapter
- **code_examples**: array of objects - Code examples in the chapter
  - language: string (e.g., "python", "cpp", "yaml", "bash")
  - code: string - The actual code
  - description: string - Description of what the code does
- **diagrams**: array of strings - Mermaid diagram definitions
- **related_topics**: array of strings - Related topics/sections
- **created_at**: datetime - Creation timestamp
- **updated_at**: datetime - Last update timestamp

### CodeExample
- **id**: string (UUID) - Unique identifier for each code example
- **chapter_id**: string - Reference to the chapter containing this example
- **title**: string - Brief title of the code example
- **language**: string - Programming language (e.g., "python", "cpp", "yaml")
- **code**: string - The actual code content
- **description**: string - Explanation of the code
- **file_path**: string - Where this code should be located in the ROS workspace
- **dependencies**: array of strings - Required dependencies/packages
- **execution_notes**: string - How to run/test the code
- **validation_criteria**: string - How to verify the code works correctly

### LearningPath
- **id**: string (UUID) - Unique identifier for each learning path
- **name**: string - Name of the learning path (e.g., "Beginner Robotics", "Advanced AI Integration")
- **description**: string - Description of the learning path
- **target_audience**: string - Who this path is designed for
- **chapters**: array of objects - Ordered list of chapters in the path
  - chapter_id: string - Reference to the chapter
  - required: boolean - Whether this chapter is required or optional
- **estimated_duration**: string - Estimated time to complete the path
- **prerequisites**: array of strings - Prerequisites for starting this path

### ChatSession
- **id**: string (UUID) - Unique identifier for each chat session
- **user_id**: string - Reference to the user
- **title**: string - Title of the chat session
- **created_at**: datetime - When the session was created
- **updated_at**: datetime - Last activity timestamp

### ChatMessage
- **id**: string (UUID) - Unique identifier for each message
- **session_id**: string - Reference to the chat session
- **sender**: string ("user" or "assistant") - Who sent the message
- **content**: string - The message content
- **timestamp**: datetime - When the message was sent
- **context_chunks**: array of strings - Context chunks used for RAG response
- **feedback**: object (optional) - User feedback on the response
  - rating: integer (1-5) - Quality rating
  - comment: string - Additional feedback

### SearchQuery
- **id**: string (UUID) - Unique identifier for each search
- **user_id**: string (optional) - Reference to the user (null for anonymous)
- **query**: string - The search query
- **results_count**: integer - Number of results returned
- **results**: array of objects - Search results
  - chapter_id: string - Reference to the chapter
  - chapter_title: string - Title of the chapter
  - snippet: string - Relevant snippet from the chapter
  - score: float - Relevance score
- **timestamp**: datetime - When the search was performed
- **response_time**: float - Time taken to return results (in seconds)

## Relationships

1. **User** → **ChatSession** (one-to-many): A user can have multiple chat sessions
2. **ChatSession** → **ChatMessage** (one-to-many): A chat session contains multiple messages
3. **DocumentationChapter** → **CodeExample** (one-to-many): A chapter can contain multiple code examples
4. **LearningPath** → **DocumentationChapter** (many-to-many through chapters array): Learning paths contain multiple chapters

## Validation Rules

1. **User**: Email must be valid and unique; profile fields must be from predefined options
2. **DocumentationChapter**: Chapter number must be between 1-12; content must be valid Markdown
3. **CodeExample**: Language must be from supported list; code must be syntactically valid for the language
4. **LearningPath**: Chapters must exist in the system; path must contain at least one required chapter
5. **ChatMessage**: Content must not exceed 10,000 characters; sender must be either "user" or "assistant"
6. **SearchQuery**: Query must be at least 2 characters and not exceed 500 characters

## State Transitions

1. **User Profile**: Can be updated by the user at any time to reflect changing experience levels
2. **ChatSession**: Created when user starts a new conversation; updated as messages are added; can be archived after period of inactivity