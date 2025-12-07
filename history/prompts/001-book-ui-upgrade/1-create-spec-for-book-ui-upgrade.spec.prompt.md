---
id: 1
title: Create spec for book UI upgrade
stage: spec
date: 2025-12-07
surface: agent
model: unknown
feature: 001-book-ui-upgrade
branch: 001-book-ui-upgrade
user: unknown
command: /sp.specify "You are updating an existing Docusaurus-based technical book project titled “Physical AI & Humanoid Robotics.” The book structure, sidebar, category JSONs, and previous content must remain fully intact. Focus only on upgrading the UI, styling, and presentation of content in a professional, readable, and visually appealing way for a technical audience. Content to enhance is already in the /docs folder and includes sections: Quarter Overview, Modules 1–4, Why Physical AI Matters, Learning Outcomes, Weekly Breakdown, Assessments, Hardware Requirements, Lab Options, Cloud Options, and Latency Trap. Your tasks: 1. **Typography & Readability** - Use professional, modern fonts for headers, subheaders, and body text. - Distinguish headers, subheaders, and sub-subheaders clearly. - Improve spacing and line height for long technical paragraphs. 2. **Module & Section Layout** - Add clean, card-like boxes for each Module with a brief description. - Highlight key points using visually distinct callout boxes. - Include icons for ROS 2, Gazebo, Unity, NVIDIA Isaac, LLMs, and Robotics modules where appropriate. 3. **Tables & Data** - Format all hardware tables, component lists, and cost breakdowns with styled tables. - Use alternating row colors, bold headers, and aligned text for easy reading. 4. **Visual Emphasis** - Highlight key terms like "ROS 2," "Isaac Sim," "Jetson Orin," "VLA," and "Sim-to-Real" using bold, colored text or badges. - Add section dividers to separate major modules and topics. - Include code block styling for commands (e.g., ROS 2 commands, Python snippets). 5. **Callouts & Notes** - Add “Tip,” “Warning,” and “Note” boxes for important technical instructions (e.g., GPU requirements, latency warnings, URDF notes). - Make these visually distinct with color-coded backgrounds. 6. **Lists & Breakdowns** - Convert long paragraphs of hardware specifications into numbered or bulleted lists. - Make multi-step instructions visually clear using nested lists or step numbers. 7. **Navigation & Flow** - Ensure sidebar links match module headings exactly. - Maintain backward compatibility: no changes to existing file names, paths, or JSON configurations. 8. **Professional Finishing** - Include subtle shadowing or card styling for sections. - Use consistent accent colors for headers, highlights, and callouts. - Ensure mobile responsiveness and proper alignment on all screen sizes. **Output**: Only modify /docs content. Generate a fully styled Docusaurus-ready markdown version of your book content with all the above premium enhancements applied. Ensure it is visually modern, professional, and highly readable, while keeping the original structure and information intact. End of Prompt."
labels: [spec, book-ui-upgrade]
links:
  spec: specs/001-book-ui-upgrade/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-book-ui-upgrade/spec.md
  - specs/001-book-ui-upgrade/checklists/requirements.md
tests:
  - None
---

## Prompt

You are updating an existing Docusaurus-based technical book project titled “Physical AI & Humanoid Robotics.” The book structure, sidebar, category JSONs, and previous content must remain fully intact. Focus only on upgrading the UI, styling, and presentation of content in a professional, readable, and visually appealing way for a technical audience. Content to enhance is already in the /docs folder and includes sections: Quarter Overview, Modules 1–4, Why Physical AI Matters, Learning Outcomes, Weekly Breakdown, Assessments, Hardware Requirements, Lab Options, Cloud Options, and Latency Trap. Your tasks: 1. **Typography & Readability** - Use professional, modern fonts for headers, subheaders, and body text. - Distinguish headers, subheaders, and sub-subheaders clearly. - Improve spacing and line height for long technical paragraphs. 2. **Module & Section Layout** - Add clean, card-like boxes for each Module with a brief description. - Highlight key points using visually distinct callout boxes. - Include icons for ROS 2, Gazebo, Unity, NVIDIA Isaac, LLMs, and Robotics modules where appropriate. 3. **Tables & Data** - Format all hardware tables, component lists, and cost breakdowns with styled tables. - Use alternating row colors, bold headers, and aligned text for easy reading. 4. **Visual Emphasis** - Highlight key terms like "ROS 2," "Isaac Sim," "Jetson Orin," "VLA," and "Sim-to-Real" using bold, colored text or badges. - Add section dividers to separate major modules and topics. - Include code block styling for commands (e.g., ROS 2 commands, Python snippets). 5. **Callouts & Notes** - Add “Tip,” “Warning,” and “Note” boxes for important technical instructions (e.g., GPU requirements, latency warnings, URDF notes). - Make these visually distinct with color-coded backgrounds. 6. **Lists & Breakdowns** - Convert long paragraphs of hardware specifications into numbered or bulleted lists. - Make multi-step instructions visually clear using nested lists or step numbers. 7. **Navigation & Flow** - Ensure sidebar links match module headings exactly. - Maintain backward compatibility: no changes to existing file names, paths, or JSON configurations. 8. **Professional Finishing** - Include subtle shadowing or card styling for sections. - Use consistent accent colors for headers, highlights, and callouts. - Ensure mobile responsiveness and proper alignment on all screen sizes. **Output**: Only modify /docs content. Generate a fully styled Docusaurus-ready markdown version of your book content with all the above premium enhancements applied. Ensure it is visually modern, professional, and highly readable, while keeping the original structure and information intact. End of Prompt.

## Response snapshot

Created spec for 001-book-ui-upgrade and validated it against the checklist.

## Outcome

- ✅ Impact: Spec created and validated.
- 🧪 Tests: No new tests added.
- 📁 Files: `spec.md` and `requirements.md` created.
- 🔁 Next prompts: /sp.plan
- 🧠 Reflection: Handled shell script failures by using agent-native file operations.

## Evaluation notes (flywheel)

- Failure modes observed: `bash` and `grep` not found; command rejected due to long argument string.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
