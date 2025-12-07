# Research Findings

This document summarizes the findings from the research tasks.

## 1. ROS 2 and Gazebo Versions

*   **ROS 2:** The latest stable version is **Jazzy Jalisco (LTS)**, released in May 2024 and supported until May 2029. Another active LTS release is **Humble Hawksbill**, supported until May 2027. For this project, we will target **Jazzy Jalisco** to use the latest features and have the longest support window.
*   **Gazebo:** For ROS 2 Jazzy Jalisco, the recommended Gazebo version is **Gazebo Harmonic**. Gazebo Classic (Gazebo 11) is not officially supported and has reached its end-of-life. We will use **Gazebo Harmonic**.

## 2. NVIDIA Isaac

The NVIDIA Isaac landscape has evolved from a single SDK to a suite of tools. For this project, the most relevant are:

*   **Isaac Sim:** A powerful robotics simulator. The latest recommended version is **5.1.0**. It has significant hardware requirements, including a powerful NVIDIA RTX GPU (e.g., RTX 4080 or better) with at least 16GB of VRAM, 32GB of RAM, and runs on Ubuntu 22.04 with Python 3.11.
*   **Isaac ROS:** A collection of hardware-accelerated packages for ROS 2. It is compatible with ROS 2 Jazzy and can run on both x86_64 machines with NVIDIA GPUs and on NVIDIA Jetson platforms.

The book will cover using both Isaac Sim for advanced simulation and Isaac ROS for accelerating perception and other tasks on hardware.

## 3. Docusaurus Version

*   The latest stable version of Docusaurus is **3.9.2**. We will use this version to build the book website.

## 4. VLA and Whisper Models

*   **Visual Language Assistant (VLA) Models:**
    *   **Open-Source:** There are several options for local deployment, such as **LLaVA**, **Idefics2**, and **MiniCPM-V 2.6**. For more powerful, larger models, there are **Gemma**, **Molmo**, and **Qwen3-VL**.
    *   **API-based:** **OpenAI's GPT-4V** and **NVIDIA's NIM** offer state-of-the-art performance through an API.
    *   **Book Strategy:** The book will demonstrate how to use a smaller, locally runnable model like **LLaVA** for accessibility, and also show how to integrate with a powerful API like **GPT-4V** for cutting-edge performance.
*   **Whisper Models:**
    *   **OpenAI's Whisper** is the leading open-source model for speech-to-text. It is available in various sizes (tiny, base, small, medium, large).
    *   **Optimized Versions:** For faster performance, there are optimized versions like **Faster Whisper** and **Distil-Whisper**.
    *   **Book Strategy:** The book will show how to use a smaller Whisper model for real-time applications on edge devices, and a larger model for more accurate transcription when resources allow. It will also cover using the OpenAI API for Whisper.

## 5. Testing Frameworks

*   **ROS 2 Nodes:**
    *   **Unit Tests:** `gtest` for C++ and `pytest` for Python.
    *   **Integration Tests:** `launch_testing` will be used to test the interaction between multiple nodes.
*   **Gazebo Simulations:**
    *   Simulations will be run in **headless mode** in CI.
    *   We will use `gz::sim::TestFixture` for scripted, automated tests.
    *   **rosbags** will be used to record and playback real-world data to improve simulation fidelity.
*   **Docusaurus Site:**
    *   The build will be checked with `npm run build`.
    *   We will configure Docusaurus to fail on broken links.
    *   Local development will use `npm start` with hot-reloading.

## 6. Edge Devices

For deploying the capstone project, the book will provide guidance on a range of hardware with a "good, better, best" approach:

*   **Good (Cost-Effective):** **Raspberry Pi 4** with a **Google Coral USB Accelerator**. This is a budget-friendly option for basic robotics and AI tasks.
*   **Better (Balanced):** **NVIDIA Jetson Orin Nano**. This offers a significant performance boost over the Raspberry Pi and is a great mid-range option.
*   **Best (High-Performance):** **NVIDIA Jetson AGX Orin**. This is a high-end option for advanced robotics, capable of running complex AI models and simulations.

The book will include instructions and code modifications for each of these platforms where applicable.

## 7. Performance Goals

*   **ROS 2 Simulations:** The goal is to achieve a **real-time factor (RTF) of 1.0 or higher** for the simulations in the book on the recommended hardware. Performance will be measured using tools like `ros2_tracing`.
*   **Docusaurus Website:** The website should achieve a **Lighthouse score of 90+** for performance, accessibility, best practices, and SEO. Build times should be kept under 2 minutes.