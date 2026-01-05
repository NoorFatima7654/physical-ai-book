# Data Model: Docusaurus Book Content Structure

## Entities

### Book
- **Definition**: The top-level container for all chapters and lessons
- **Attributes**:
  - title (string): The main title of the book
  - description (string): Brief description of the book's content
  - author (string): Author or organization name
  - version (string): Version identifier for the book
- **Relationships**: Contains multiple Chapters

### Chapter
- **Definition**: A major section of the book containing multiple lessons
- **Attributes**:
  - id (string): Unique identifier for the chapter
  - title (string): Display title of the chapter
  - position (integer): Order in the book sequence
  - description (string): Brief overview of chapter content
- **Relationships**: Contains multiple Lessons; belongs to one Book

### Lesson
- **Definition**: An individual topic or unit within a chapter
- **Attributes**:
  - id (string): Unique identifier for the lesson
  - title (string): Display title of the lesson
  - position (integer): Order within the chapter
  - content (string): The main content in Markdown format
  - prerequisites (array of strings): Other lessons that should be completed first
- **Relationships**: Belongs to one Chapter

### NavigationItem
- **Definition**: Represents an item in the site navigation
- **Attributes**:
  - type (string): Either "category" (for chapters) or "doc" (for lessons)
  - label (string): Display text in navigation
  - id (string): Reference to the document or category
  - position (integer): Order in navigation
- **Relationships**: Links to Chapters and Lessons

## Content File Structure

### Chapter Directory (_category_.json)
- **Purpose**: Defines metadata for a chapter
- **Location**: docs/[chapter-name]/_category_.json
- **Fields**:
  - label: Display name for the chapter
  - position: Ordering in sidebar
  - collapsible: Whether the chapter section can be collapsed (default: true)
  - collapsed: Initial state of the chapter section (default: false)

### Lesson File (lesson-name.md)
- **Purpose**: Contains the content for a specific lesson
- **Location**: docs/[chapter-name]/lesson-name.md
- **Frontmatter Fields**:
  - id: Unique identifier (defaults to filename if not specified)
  - title: Display title for the lesson
  - sidebar_label: Alternative title for sidebar navigation
  - sidebar_position: Order of this lesson within the chapter
  - description: Brief summary of lesson content

## Relationships

1. **Book-Chapter**: One-to-Many relationship
   - A Book contains multiple Chapters
   - Each Chapter belongs to exactly one Book

2. **Chapter-Lesson**: One-to-Many relationship
   - A Chapter contains multiple Lessons
   - Each Lesson belongs to exactly one Chapter

3. **Navigation-Content**: Many-to-Many relationship through configuration
   - NavigationItems reference Chapters and Lessons
   - The sidebar configuration determines the navigation structure