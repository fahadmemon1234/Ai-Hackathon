# Data Model

This document describes the key data entities for the Physical AI & Humanoid Robotics book.

## Entities

### 1. Module/Chapter

A self-contained learning unit.

| Field | Type | Description |
|---|---|---|
| `id` | String | Unique identifier for the module (e.g., `module-1-foundations`). |
| `title` | String | The title of the module (e.g., "Module 1: Foundations of Physical AI"). |
| `description` | String | A brief summary of the module's content. |
| `content` | Markdown | The full content of the module, written in Docusaurus-ready Markdown. |
| `examples` | Array<CodeSnippet> | A list of code examples included in the module. |
| `diagrams` | Array<Diagram/Image> | A list of diagrams and images used in the module. |
| `tables` | Array<Table> | A list of tables used in the module. |
| `exercises` | Array<String> | A list of exercises for the reader to complete. |

### 2. Code Snippet

A block of code that demonstrates a specific concept or task.

| Field | Type | Description |
|---|---|---|
| `id` | String | Unique identifier for the code snippet. |
| `title` | String | A brief title for the code snippet. |
| `language` | String | The programming language of the snippet (e.g., `python`, `cpp`, `xml`). |
| `code` | String | The actual code content. |
| `explanation` | Markdown | A description of what the code does. |
| `runnable` | Boolean | Whether the code snippet is runnable as a standalone example. |

### 3. Diagram/Image

A visual aid to help explain a concept or setup.

| Field | Type | Description |
|---|---|---|
| `id` | String | Unique identifier for the diagram/image. |
| `src` | String | The path to the image file. |
| `alt` | String | The alt text for the image. |
| `caption` | String | A caption for the image. |

### 4. Table

A structured presentation of information.

| Field | Type | Description |
|---|---|---|
| `id` | String | Unique identifier for the table. |
| `title` | String | The title of the table. |
| `data` | Array<Array<String>> | The table data, represented as a 2D array of strings. |
| `caption` | String | A caption for the table. |

### 5. Capstone Project

A final project that integrates the concepts from all modules.

| Field | Type | Description |
|---|---|---|
| `id` | String | Unique identifier for the capstone project. |
| `title` | String | The title of the capstone project. |
| `description` | Markdown | A detailed description of the project, including goals and requirements. |
| `steps` | Array<String> | A list of steps to complete the project. |
| `code_snippets` | Array<CodeSnippet> | A list of code snippets for the project. |
| `diagrams` | Array<Diagram/Image> | A list of diagrams and images for the project. |

## Relationships

*   A **Module/Chapter** can have multiple **Code Snippets**, **Diagrams/Images**, and **Tables**.
*   The **Capstone Project** is a special entity that brings together concepts from all **Modules/Chapters** and has its own set of **Code Snippets** and **Diagrams/Images**.
