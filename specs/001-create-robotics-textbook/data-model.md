# Data Model: AI-Native Textbook for Physical AI & Humanoid Robotics

This document defines the data model for the project, based on the entities identified in the feature specification.

## Entities

### User

Represents a learner in the system.

- **`user_id`**: `UUID` (Primary Key)
- **`email`**: `String` (Unique, Indexed)
- **`password_hash`**: `String`
- **`software_background`**: `Enum` (`Beginner`, `Intermediate`, `Advanced`)
- **`hardware_experience`**: `Enum` (`None`, `Arduino/RPi`, `ROS`, `Robotics Kit`)
- **`preferred_learning`**: `Enum` (`Visual`, `Code-heavy`, `Theory`, `Hands-on`)
- **`created_at`**: `Timestamp`
- **`updated_at`**: `Timestamp`

### Module

A top-level section of the textbook.

- **`module_id`**: `UUID` (Primary Key)
- **`title`**: `String`
- **`description`**: `Text`
- **`order`**: `Integer`

### Chapter

A subsection of a module.

- **`chapter_id`**: `UUID` (Primary Key)
- **`module_id`**: `UUID` (Foreign Key to `Module`)
- **`title`**: `String`
- **`content`**: `Text` (Markdown format)
- **`order`**: `Integer`

### ChatMessage

A record of a user's interaction with the RAG chatbot.

- **`message_id`**: `UUID` (Primary Key)
- **`user_id`**: `UUID` (Foreign Key to `User`)
- **`session_id`**: `UUID` (Indexed)
- **`query`**: `Text`
- **`response`**: `Text`
- **`timestamp`**: `Timestamp`

## Relationships

- A `User` can have many `ChatMessage`s.
- A `Module` can have many `Chapter`s.
- A `Chapter` belongs to one `Module`.
- A `ChatMessage` belongs to one `User`.
