---

description: "Task list for Book UI Upgrade feature implementation"
---

# Tasks: Book UI Upgrade

**Input**: Design documents from `/specs/001-book-ui-upgrade/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: No explicit test tasks are generated as the feature spec did not request a TDD approach. Testing will primarily involve visual verification and manual checks as described in quickstart.md.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story?] Description with file path`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Web app**: `book/` at repository root
- Paths shown below assume Docusaurus project structure

## Phase 1: Setup (Project Initialization & Configuration)

**Purpose**: Ensure the Docusaurus project is ready for custom styling.

- [x] T001 Verify Docusaurus project structure and `package.json` dependencies in `book/`
- [x] T002 Locate `custom.css` (or create if missing) in `book/src/css/custom.css`
- [x] T003 Ensure `docusaurus.config.ts` references `custom.css` correctly in `book/docusaurus.config.ts`
- [x] T004 Run `npm install` in `book/` to ensure all dependencies are up to date
- [x] T005 Run `npm run start` in `book/` to verify local development server starts without errors

---

## Phase 2: Foundational (Basic Styling Infrastructure)

**Purpose**: Establish the core custom styling capabilities before detailed UI work.

- [x] T006 Define root CSS variables for consistent accent colors in `book/src/css/custom.css`
- [x] T007 Implement basic reset/normalize styles if needed in `book/src/css/custom.css`
- [x] T008 Add base styles for `body` and `html` for consistent typography settings across the book in `book/src/css/custom.css`

---

## Phase 3: User Story 1 - Enhanced Content Readability (P1) 🎯 MVP

**Goal**: Implement professional and modern typography, clear heading distinction, and improved line spacing for better content readability.

**Independent Test**: Visually verify any documentation page (e.g., `book/docs/01-intro-physical-ai.md`) to confirm typography, spacing, and heading hierarchy.

### Implementation for User Story 1

- [x] T009 [US1] Define and apply modern fonts for `h1`, `h2`, `h3`, `h4`, `h5`, `h6` elements in `book/src/css/custom.css`
- [x] T010 [P] [US1] Define and apply modern fonts for body text and paragraphs (`p`) in `book/src/css/custom.css`
- [x] T011 [P] [US1] Adjust `line-height` and `margin` for `p` elements for improved readability of long technical paragraphs in `book/src/css/custom.css`
- [x] T012 [P] [US1] Create distinct styles (font size, weight, color) for `h1` and `h2` elements in `book/src/css/custom.css`
- [x] T013 [P] [US1] Create distinct styles (font size, weight, color) for `h3` and `h4` elements in `book/src/css/custom.css`
- [x] T014 [P] [US1] Create distinct styles (font size, weight, color) for `h5` and `h6` elements in `book/src/css/custom.css`

---

## Phase 4: User Story 7 - Consistent and Responsive UI (P1)

**Goal**: Ensure a consistent, professional, and mobile-responsive UI with subtle finishing touches across all devices.

**Independent Test**: Navigate various pages of the book on different screen sizes (browser resize, mobile device emulator) to verify responsiveness, consistent accent colors, and card styling.

### Implementation for User Story 7

- [x] T015 [US7] Implement subtle shadowing or card styling for relevant sections/containers (e.g., Markdown-rendered blocks) in `book/src/css/custom.css`
- [x] T016 [US7] Apply consistent accent colors (using defined CSS variables) to headers, highlights, and callout elements in `book/src/css/custom.css`
- [x] T017 [US7] Implement responsive design principles using media queries to ensure proper alignment and readability on small screens in `book/src/css/custom.css`
- [x] T018 [US7] Perform a general visual audit to ensure consistent spacing and alignment across different content types in `book/src/css/custom.css`

---

## Phase 5: User Story 2 - Clear Module and Section Presentation (P1)

**Goal**: Present modules and key information in visually distinct and organized ways using card-like boxes, callout boxes, and relevant icons.

**Independent Test**: Review a module overview page (e.g., `book/docs/module1/01-content.md`) and any section with callouts to confirm card-like module presentation, distinct callouts, and icon presence.

### Implementation for User Story 2

- [x] T019 [US2] Create CSS for card-like boxes for modules, including padding, borders, and shadows in `book/src/css/custom.css`
- [x] T020 [P] [US2] Create or modify a Docusaurus component (if necessary) to wrap module content in card-like boxes (e.g., `book/src/components/ModuleCard/index.tsx`)
- [x] T021 [P] [US2] Implement CSS for visually distinct callout boxes for key points (e.g., `.callout-info`, `.callout-warning`) in `book/src/css/custom.css`
- [x] T022 [US2] Identify and integrate appropriate SVG icons for technologies (ROS 2, Gazebo, Unity, NVIDIA Isaac, LLMs, Robotics) in `book/static/img/` or by referencing a component.
- [x] T023 [US2] Develop or modify Docusaurus components/markdown plugins to display icons alongside technology names or module descriptions.

---

## Phase 6: User Story 3 - Professional Data Presentation (P2)

**Goal**: Format all hardware tables, component lists, and cost breakdowns with styled tables for improved readability.

**Independent Test**: Verify any table in the documentation (e.g., hardware specifications, component lists) to confirm bold headers, alternating row colors, and aligned text.

### Implementation for User Story 3

- [x] T024 [US3] Apply base CSS for `table` elements, including borders, padding, and text alignment in `book/src/css/custom.css`
- [x] T025 [US3] Implement styling for `th` (table headers) to be bold and clearly distinguishable in `book/src/css/custom.css`
- [x] T026 [US3] Implement alternating row colors for `tr` elements within `table` using `:nth-child(even)` or similar in `book/src/css/custom.css`

---

## Phase 7: User Story 4 - Visual Cues for Key Information (P2)

**Goal**: Visually emphasize important terms, separate major topics with section dividers, and style code blocks distinctly.

**Independent Test**: Review various documentation pages for highlighted terms, presence of section dividers, and distinct styling of code blocks.

### Implementation for User Story 4

- [x] T027 [US4] Define CSS for badges or inline styling to highlight key terms like "ROS 2," "Isaac Sim," "Jetson Orin," "VLA," "Sim-to-Real" with bold and accent colors in `book/src/css/custom.css`
- [x] T028 [US4] Implement CSS for aesthetically pleasing section dividers (`<hr>`) to visually separate major modules and topics in `book/src/css/custom.css`
- [x] T029 [US4] Customize Docusaurus code block styling (background, font, padding) for commands and Python snippets in `book/src/css/custom.css`

---

## Phase 8: User Story 5 - Actionable Callouts and Notes (P2)

**Goal**: Implement visually distinct "Tip," "Warning," and "Note" boxes with color-coded backgrounds for important technical instructions.

**Independent Test**: Locate examples of Tip, Warning, and Note boxes in documentation and confirm their distinct color-coded backgrounds and visual emphasis.

### Implementation for User Story 5

- [x] T030 [US5] Implement CSS for a "Tip" callout box with a distinct background color (e.g., green) and icon in `book/src/css/custom.css`
- [x] T031 [P] [US5] Implement CSS for a "Warning" callout box with a distinct background color (e.g., yellow/orange) and icon in `book/src/css/custom.css`
- [x] T032 [P] [US5] Implement CSS for a "Note" callout box with a distinct background color (e.g., blue) and icon in `book/src/css/custom.css`

---

## Phase 9: User Story 6 - Structured Lists and Breakdowns (P2)

**Goal**: Convert long paragraphs of hardware specifications into readable numbered or bulleted lists and make multi-step instructions visually clear.

**Independent Test**: Review sections containing hardware specifications and multi-step instructions to verify their presentation as structured lists.

### Implementation for User Story 6

- [x] T033 [US6] Apply base styling for `ul` and `ol` elements for improved readability, including spacing and bullet/number styles in `book/src/css/custom.css`
- [x] T034 [US6] Ensure nested lists are visually distinct and correctly indented in `book/src/css/custom.css`

---

## Final Phase: Polish & Cross-Cutting Concerns

**Purpose**: Final review, performance optimization, accessibility checks, and overall quality assurance.

- [x] T035 Perform a final comprehensive visual audit across the entire book to ensure consistency and adherence to all UI/UX goals.
- [x] T036 Validate mobile responsiveness on actual devices or comprehensive browser emulation tools.
- [x] T037 Run `npm run build` in `book/` to ensure a production-ready build can be generated without errors.
- [x] T038 Review `quickstart.md` and perform all steps to validate the implemented changes.
- [x] T039 Document any new Docusaurus component usages or custom CSS classes in a dedicated file (e.g., `book/docs/styling-guide.md`).

---

## Dependencies & Execution Order

### Phase Dependencies

-   **Phase 1: Setup**: No dependencies - can start immediately.
-   **Phase 2: Foundational**: Depends on Phase 1 completion - establishes core styling.
-   **User Stories (Phase 3-9)**: All depend on Phase 2 completion.
-   **Final Phase: Polish**: Depends on all user story phases being substantially complete.

### User Story Dependencies

The user stories are designed to be largely independent in terms of implementation flow, allowing for parallel work. However, certain styling aspects from earlier phases (e.g., base typography from US1, general responsiveness from US7) will naturally form a visual foundation for subsequent stories.

-   **User Story 1 (P1)**: Can start after Foundational (Phase 2). No direct dependencies on other user stories.
-   **User Story 7 (P1)**: Can start after Foundational (Phase 2). Integrates closely with all other UI changes to ensure consistency and responsiveness.
-   **User Story 2 (P1)**: Can start after Foundational (Phase 2).
-   **User Story 3 (P2)**: Can start after Foundational (Phase 2).
-   **User Story 4 (P2)**: Can start after Foundational (Phase 2).
-   **User Story 5 (P2)**: Can start after Foundational (Phase 2).
-   **User Story 6 (P2)**: Can start after Foundational (Phase 2).

### Within Each User Story

-   Styling tasks within each user story should be implemented, followed by visual verification.
-   Tasks related to creating or modifying Docusaurus components should precede the application of their styles.

### Parallel Opportunities

-   **Within Phases**: Tasks marked with `[P]` within a phase can generally be executed in parallel as they often involve modifying different sections of the `custom.css` or distinct component files.
-   **Across User Stories**: Once the Foundational Phase is complete, different user stories can be worked on in parallel by different developers or in interleaved fashion, as most changes are styling-oriented and should merge with minimal conflicts if focused on distinct CSS classes or components.

---

## Parallel Example: User Story 1 & 7 (Initial P1 Focus)

```bash
# User Story 1 (Typography)
Task: "Define and apply modern fonts for h1, h2, h3, h4, h5, h6 elements in book/src/css/custom.css"
Task: "Define and apply modern fonts for body text and paragraphs (p) in book/src/css/custom.css"
Task: "Adjust line-height and margin for p elements for improved readability of long technical paragraphs in book/src/css/custom.css"

# User Story 7 (Consistency & Responsiveness)
Task: "Implement subtle shadowing or card styling for relevant sections/containers (e.g., Markdown-rendered blocks) in book/src/css/custom.css"
Task: "Apply consistent accent colors (using defined CSS variables) to headers, highlights, and callout elements in book/src/css/custom.css"
Task: "Implement responsive design principles using media queries to ensure proper alignment and readability on small screens in book/src/css/custom.css"
```

---

## Implementation Strategy

### MVP First (Prioritize P1 User Stories)

1.  Complete Phase 1: Setup.
2.  Complete Phase 2: Foundational (CRITICAL - blocks all user stories).
3.  Implement tasks for User Story 1 (Enhanced Content Readability).
4.  Implement tasks for User Story 7 (Consistent and Responsive UI) - as it forms a visual foundation for all other stories.
5.  Implement tasks for User Story 2 (Clear Module and Section Presentation).
6.  **STOP and VALIDATE**: Visually verify the enhanced typography, basic responsiveness, module presentation, and overall consistency. Ensure the core reading experience is significantly improved.
7.  Deploy/demo if ready for early feedback.

### Incremental Delivery

1.  Complete Setup + Foundational → Foundation ready for styling.
2.  Add User Story 1 & 7 → Test independently (visual verification, responsiveness check) → Deploy/Demo (Core UI improved).
3.  Add User Story 2 → Test independently → Deploy/Demo (Module presentation enhanced).
4.  Add User Story 3 → Test independently → Deploy/Demo (Tables styled).
5.  Continue with remaining P2 stories (US4, US5, US6) in sequence or parallel.
6.  Each story adds value without breaking previous stories, allowing for continuous integration and delivery.

### Parallel Team Strategy

With multiple developers:

1.  Team completes Setup + Foundational together.
2.  Once Foundational is done:
    *   **Developer A**: Focus on User Story 1 (Typography) and User Story 7 (Responsiveness & Consistency).
    *   **Developer B**: Focus on User Story 2 (Module & Section Layout), including component creation and icon integration.
    *   **Developer C**: Focus on User Story 3 (Tables) and User Story 4 (Visual Cues).
    *   **Developer D**: Focus on User Story 5 (Callouts) and User Story 6 (Lists).
3.  Regular merges and visual reviews are crucial to ensure consistency.

---

## Notes

-   `[P]` tasks = different files, no dependencies within the same user story or phase.
-   `[Story]` label maps task to specific user story for traceability.
-   Each user story (or group of closely related stories like P1s) should be independently completable and testable.
-   Visual verification is key throughout the process.
-   Commit after each task or logical group.
-   Stop at any checkpoint to validate story independently.
-   Avoid: vague tasks, same file conflicts (CSS changes should be additive or use specific selectors), cross-story dependencies that break independence.
