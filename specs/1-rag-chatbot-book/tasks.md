# Tasks for Integrated RAG Chatbot

**Feature Branch**: `1-rag-chatbot-book`
**Created**: 2025-12-17
**Plan**: [plan.md](./plan.md)
**Specification**: [spec.md](./spec.md)

## Summary

This document outlines the actionable tasks for implementing the Integrated RAG Chatbot feature. Tasks are organized by phases, primarily following the user stories and their priorities, ensuring incremental and independently testable delivery.

## Task Dependency Graph

```mermaid
graph TD
    subgraph Setup
        T001 --> T002
        T002 --> T003
        T003 --> T004
        T004 --> F001
    end

    subgraph Foundational
        F001 --> F002
        F002 --> F003
        F003 --> F004
        F004 --> F005
        F005 --> US1_T001
    end

    subgraph User Story 1 (P1): Core RAG Chatbot (Entire Book)
        US1_T001 --> US1_T002
        US1_T002 --> US1_T003
        US1_T003 --> US1_T004
        US1_T004 --> US1_T005
        US1_T005 --> US1_T006
        US1_T006 --> US2_T001
    end

    subgraph User Story 2 (P2): Selected-Text Contextual RAG
        US2_T001 --> US2_T002
        US2_T002 --> US2_T003
        US2_T003 --> US2_T004
        US2_T004 --> P001
    end

    subgraph Polish & Cross-Cutting Concerns
        P001 --> P002
        P002 --> P003
        P003 --> P004
    end
```

## Parallel Execution Opportunities

-   **User Story 1**:
    -   `US1_T001`, `US1_T002`, `US1_T003` could potentially be parallel if dependencies are carefully managed (e.g., Qdrant service is independent of agent service core).
    -   Frontend development (`US1_T006`) can run in parallel with some backend tasks.
-   **User Story 2**:
    -   Frontend enhancements (`US2_T001`, `US2_T002`) can be developed in parallel with backend extensions (`US2_T003`, `US2_T004`).

## Implementation Strategy

The implementation will follow an iterative approach, delivering the highest priority user stories first (MVP-first). Each user story is designed to be independently testable.

## Phases & Tasks

### Phase 1: Setup

Goal: Initialize the project environment and basic structure.

- [X] T001 Create `backend/` directory structure: `src/`, `src/api/`, `src/core/`, `src/services/`, `src/models/`, `tests/`
- [X] T002 Initialize `backend/src/__init__.py`, `backend/src/api/__init__.py`, `backend/src/core/__init__.py`, `backend/src/services/__init__.py`, `backend/src/models/__init__.py`
- [X] T003 Create base `backend/src/core/config.py` for environment variables and settings
- [X] T004 Create `requirements.txt` in `backend/` with initial dependencies: `fastapi`, `uvicorn`, `python-dotenv`, `qdrant-client`, `cohere`, `python-sdk`, `asyncpg`, `pytest`, `python-gemini-sdk`

### Phase 2: Foundational

Goal: Establish core backend services and the book ingestion pipeline, essential for any RAG functionality.

- [X] F001 Implement `backend/src/core/dependencies.py` for database, Qdrant, and LLM client dependency injection
- [X] F002 Develop `backend/src/services/qdrant_service.py` to handle Qdrant connection and basic vector operations
- [X] F003 Implement `backend/src/models/schemas.py` for `BookContentChunk` data model based on `data-model.md`
- [X] F004 Develop book ingestion script in `backend/src/ingestion_script.py` (chunking, embedding with Cohere, storing in Qdrant)
- [X] F005 Create `backend/src/main.py` for the FastAPI application, including basic app setup

### Phase 3: User Story 1 (P1) - Core RAG Chatbot (Entire Book)

Goal: Enable users to ask questions about the entire book content and receive relevant answers.

Independent Test: Can be tested by opening the chat interface, asking a question related to the book's content, and verifying that a relevant answer is returned.

- [X] US1_T001 Implement `backend/src/services/agent_service.py` core logic (orchestration of RAG, prompt construction for Gemini)
- [X] US1_T002 Update `backend/src/services/agent_service.py` to integrate with Qdrant service for context retrieval (full-book queries)
- [X] US1_T003 Update `backend/src/services/agent_service.py` for Gemini invocation using `python-gemini-sdk`
- [X] US1_T004 Develop `backend/src/api/chat.py` to handle `POST /api/chat` endpoint (full-book query logic, chat session handling with Neon)
- [X] US1_T005 Implement `backend/tests/test_chat_api.py` for `POST /api/chat` (full-book) integration tests
- [X] US1_T006 Develop `book/src/components/ChatbotWidget.jsx` for basic chat UI and `book/src/components/ChatbotWidget.module.css`

### Phase 4: User Story 2 (P2) - Selected-Text Contextual RAG

Goal: Allow users to highlight specific text and ask contextual questions.

Independent Test: Can be tested by selecting a piece of text, triggering the "ask about selection" feature, and verifying that the answer is based only on the selected text.

- [X] US2_T001 Implement JavaScript in Docusaurus theme to capture selected text
- [X] US2_T002 Update `book/src/components/ChatbotWidget.jsx` to send selected text and context mode to backend
- [X] US2_T003 Extend `backend/src/api/chat.py` to accept `context_mode: selected_text` and `selected_text`
- [X] US2_T004 Modify `backend/src/services/agent_service.py` to prioritize selected text as context for Gemini
- [X] US2_T005 Implement `backend/tests/test_chat_api.py` for `POST /api/chat` (selected-text) integration tests

### Phase 5: Polish & Cross-Cutting Concerns

Goal: Improve overall quality, robustness, and maintainability.

- [X] P001 Refine error handling across backend API and services
- [X] P002 Implement comprehensive logging for debugging and monitoring
- [X] P003 Update `quickstart.md` with final installation steps and commands
- [X] P004 Review and update `README.md` for both `backend/` and `book/` directories

## Suggested MVP Scope

The Minimum Viable Product (MVP) should focus on completing **User Story 1: Core RAG Chatbot (Entire Book)**. This provides the primary value of answering questions based on the full book content, serving as a functional foundation for future enhancements.

## Format Validation

All tasks adhere to the strict checklist format: `- [ ] [TaskID] [P?] [Story?] Description with file path`.
