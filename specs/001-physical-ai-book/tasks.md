# Tasks: Physical AI & Humanoid Robotics Book

**Feature**: `001-physical-ai-book`
**Generated**: 2025-12-06

This document outlines the tasks required to create the Docusaurus book on Physical AI and Humanoid Robotics, based on the approved [plan.md](./plan.md) and [spec.md](./spec.md).

## Implementation Strategy

The project will be delivered incrementally, focusing on building a solid foundation first and then adding content module by module. The Minimum Viable Product (MVP) will consist of the initial Docusaurus setup and the first two modules, which will validate the core structure and content generation process.

**MVP Scope**: Phase 1 and the first two tasks of Phase 3 (Module 1 & 2).

## Phase 1: Project Setup & Initialization

These tasks set up the necessary project structure and tooling.

- [x] T001 Initialize Docusaurus project in the `book/` directory.
- [x] T002 Configure `book/docusaurus.config.js` with the book title, theme, and navigation structure.
- [x] T003 Create the initial source code directory structure: `code/ros2_ws/`, `code/isaac_sim/`.
- [x] T004 Create the initial test directory `tests/` and add a placeholder `tests/test_ros2_nodes.py`.
- [x] T005 [P] Draft the main `book/docs/intro.md` page with an introduction to the book.

## Phase 2: Foundational Content & Core Concepts

These tasks establish the core, prerequisite knowledge for the rest of the book.

- [x] T006 [US1] Write Chapter 1: Introduction to Physical AI in `book/docs/01-intro-physical-ai.md`.
- [x] T007 [US1] Write Chapter 2: Introduction to ROS 2 and Gazebo in `book/docs/02-intro-ros2-gazebo.md`.
- [x] T008 [P] [US1] Create initial code examples for basic ROS 2 publisher/subscriber in `code/ros2_ws/src/chapter2_examples/`.
- [x] T009 [US1] Create tests for the Chapter 2 code examples in `tests/test_chapter2_examples.py`.
- [x] T010 [P] Create a table of hardware requirements for different budgets (economy, high-end) in `book/docs/00-hardware-setup.md`.

## Phase 3: User Story 1 - Student Learning Path (Sequential Modules)

These tasks represent the core chapter-by-chapter content generation.

- [x] T011 [US1] Write Module 1 content on Advanced ROS 2 Concepts in `book/docs/module1/01-content.md`.
- [x] T012 [P] [US1] Develop code examples for Module 1 in `code/ros2_ws/src/module1_examples/`.
- [x] T013 [US1] Write tests for Module 1 code examples in `tests/test_module1_examples.py`.
- [x] T014 [US1] Write Module 2 content on NVIDIA Isaac Sim Basics in `book/docs/module2/01-content.md`.
- [x] T015 [P] [US1] Develop simulation examples for Module 2 in `code/isaac_sim/src/module2_examples/`.
- [x] T016 [US1] Write tests for Module 2 simulation examples in `tests/test_module2_examples.py`.
- [x] T017 [US1] Write Module 3 content on VLA and Whisper Integration in `book/docs/module3/01-content.md`.
- [x] T018 [P] [US1] Develop code for integrating a VLA model with ROS 2 in `code/ros2_ws/src/module3_examples/`.
- [x] T019 [US1] Write tests for Module 3 VLA integration in `tests/test_module3_examples.py`.

## Phase 4: User Story 3 - Hobbyist Hardware & Capstone Project

Tasks focused on physical hardware setup and the final project.

- [ ] T020 [US3] Write detailed hardware assembly guide in `book/docs/appendix/A-hardware-assembly.md`.
- [ ] T021 [US3] Define the Capstone Project: Voice-Driven Humanoid Robot in `book/docs/capstone/01-project-definition.md`.
- [ ] T022 [P] [US3] Provide complete ROS 2 code for the capstone project in `code/ros2_ws/src/capstone_project/`.
- [ ] T023 [P] [US3] Provide the Gazebo/Isaac simulation setup for the capstone project in `code/isaac_sim/src/capstone_project/`.
- [ ] T024 [US3] Write validation tests for the capstone project in `tests/test_capstone_project.py`.
- [ ] T025 [US3] Create the step-by-step tutorial for the capstone project in `book/docs/capstone/02-tutorial.md`.

## Phase 5: Polish & Cross-Cutting Concerns (US2)

Final tasks for quality assurance, discoverability, and deployment.

- [ ] T026 [US2] Generate a detailed index/glossary for the book to improve searchability.
- [ ] T027 [P] Review and test all code snippets and simulations for accuracy and clarity.
- [ ] T028 [P] Validate all Docusaurus links, diagrams, and tables across the entire book.
- [ ] T029 Create GitHub Actions workflow for deploying the Docusaurus book to GitHub Pages at `.github/workflows/deploy-book.yml`.
- [ ] T030 Final review of all content for grammatical errors and clarity.

## Dependencies

- **US1 (Student)** is the foundational story. Its tasks in Phase 2 and 3 should be prioritized.
- **US3 (Hobbyist)** depends on the completion of the core modules from US1. The capstone project (Phase 4) integrates knowledge from earlier chapters.
- **US2 (Professional)** is supported by a complete and well-structured book. The polish phase (Phase 5) is key for this user story.

**Story Completion Order**: US1 -> US3 -> US2.

## Parallel Execution

- Within each module/story phase, content writing (`.md` files) can be done in parallel with code development (`.py` files) to a certain extent. These tasks are marked with `[P]`.
- For example, in Phase 3, task `T011` (writing content) can happen concurrently with `T012` (developing code).
- In Phase 1, `T005` can be done at any time after `T001`.
