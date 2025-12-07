# Quickstart

This document provides a quick overview of how to get started with the project.

## 1. Prerequisites

*   **Git:** You will need Git to clone the repository.
*   **Node.js and Yarn:** You will need Node.js (v18 or later) and Yarn to run the Docusaurus website.
*   **Docker:** You will need Docker to run the ROS 2 and Gazebo simulations in a containerized environment.
*   **NVIDIA Container Toolkit:** If you have an NVIDIA GPU, you will need to install the NVIDIA Container Toolkit to enable GPU acceleration in Docker.

## 2. Getting Started

1.  **Clone the repository:**
    ```bash
    git clone https://github.com/your-username/hackathon-book.git
    cd hackathon-book
    ```

2.  **Run the Docusaurus website:**
    ```bash
    cd book
    yarn install
    yarn start
    ```
    This will start a local development server and open up a browser window. Most changes are reflected live without having to restart the server.

3.  **Run the simulations:**
    The code examples for the simulations are located in the `code` directory. Each example will have its own `README.md` with instructions on how to run it. Most simulations will be run using Docker Compose.
    ```bash
    cd code/ros2_ws
    docker-compose up
    ```

## 3. Contributing

Please see the `CONTRIBUTING.md` file for information on how to contribute to the project.
