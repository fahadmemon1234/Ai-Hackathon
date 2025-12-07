# Styling Guide for Book UI Upgrade

This document outlines the custom CSS classes and Docusaurus components introduced as part of the "Book UI Upgrade" feature. These elements are designed to enhance the visual presentation and readability of the book.

## Custom CSS Classes

The following CSS classes have been added to `book/src/css/custom.css` to provide enhanced styling:

*   `.module-card`: Applied to sections or components intended to display module overviews in a card-like format.
*   `.callout-tip`, `.callout-warning`, `.callout-note`: Used for visually distinct callout boxes for important information, warnings, and notes, respectively. These leverage predefined accent colors for their borders and backgrounds.
*   `.highlight-term`: Applied to key technical terms within the text to give them visual emphasis with bold text and accent colors.

## Custom Docusaurus Components

The following new React components have been introduced in `book/src/components/`:

*   `ModuleCard/index.tsx`: A component designed to wrap module content, applying the `.module-card` styling. It can optionally display a title and description.
*   `TechIcon/index.tsx`: A utility component for easily displaying SVG icons for various technologies (e.g., ROS 2, Gazebo) by referencing their `iconName` and `alt` text. Icons are expected to be located in `book/static/img/icons/`.

## Usage Examples

### Module Card

```markdown
import ModuleCard from '@site/src/components/ModuleCard';

<ModuleCard title="Introduction to Physical AI" description="An overview of Physical AI and its importance.">
  This is the content of your module.
</ModuleCard>
```

### Callout Boxes

```markdown
<div className="callout callout-tip">
  <p><b>Tip:</b> Always check your GPU drivers for optimal performance.</p>
</div>

<div className="callout callout-warning">
  <p><b>Warning:</b> Latency issues can significantly impact real-time robotics.</p>
</div>

<div className="callout callout-note">
  <p><b>Note:</b> URDF files are crucial for robot descriptions.</p>
</div>
```

### Highlighted Terms

```markdown
This section discusses the importance of <span className="highlight-term">ROS 2</span> in modern robotics.
```

### Technology Icons

```markdown
import TechIcon from '@site/src/components/TechIcon';

<TechIcon iconName="ros2" alt="ROS 2 Logo" /> ROS 2
```
