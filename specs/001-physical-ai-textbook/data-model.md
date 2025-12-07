# Data Model: Physical AI & Humanoid Robotics Textbook

## Overview
This document defines the data models for the Physical AI & Humanoid Robotics textbook application based on the entities identified in the feature specification.

## Core Entities

### Course Module
- **Description**: Represents one of the core subject areas in the textbook (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
- **Fields**:
  - id: Unique identifier for the module
  - title: Name of the module (e.g., "ROS 2", "Gazebo/Unity")
  - description: Brief overview of the module content
  - learningObjectives: Array of learning objectives for the module
  - contentPath: File path to module content
  - order: Numeric order of the module in the curriculum
- **Relationships**:
  - Contains multiple WeeklyBreakdown entities
  - Contains multiple LabExercise entities
  - Contains multiple Assessment entities
- **Validation Rules**:
  - Title must be one of the four core modules (ROS 2, Gazebo/Unity, NVIDIA Isaac, VLA)
  - Learning objectives must be clearly defined (non-empty array)
  - Order must be unique across all modules

### Weekly Breakdown
- **Description**: Organizes course content by week (1-13) with specific topics, labs, and assignments for progressive learning
- **Fields**:
  - id: Unique identifier for the weekly content
  - weekNumber: Numeric value representing the week (1-13)
  - title: Descriptive title for the weekly content
  - topics: Array of topics covered in the week
  - module: Reference to the parent Course Module
  - labExercises: Array of references to associated lab exercises
  - assessments: Array of references to associated assessments
  - estimatedHours: Estimated time required to complete the week's content
- **Relationships**:
  - Belongs to one Course Module
  - Contains multiple LabExercise entities
  - Contains multiple Assessment entities
- **Validation Rules**:
  - Week number must be between 1 and 13
  - Week number must be unique within a module
  - Topics array must not be empty
  - Estimated hours must be a positive number

### Lab Exercise
- **Description**: Hands-on activities that guide students through practical implementation of concepts using real tools and simulations
- **Fields**:
  - id: Unique identifier for the lab exercise
  - title: Descriptive title of the lab exercise
  - description: Overview of the lab exercise objectives
  - objectives: Array of specific learning objectives for the lab
  - prerequisites: Array of required knowledge or materials
  - steps: Array of ordered instructions for the lab
  - estimatedDuration: Estimated time to complete the lab (in minutes)
  - module: Reference to the parent Course Module
  - week: Reference to the associated Weekly Breakdown
  - difficulty: Enum (Beginner, Intermediate, Advanced)
  - requiredEquipment: Array of required hardware/software
- **Relationships**:
  - Belongs to one Course Module
  - Associated with one Weekly Breakdown
- **Validation Rules**:
  - Title and description must not be empty
  - Steps array must not be empty
  - Estimated duration must be a positive number
  - Difficulty must be one of the defined enum values

### Assessment
- **Description**: Tools to evaluate student comprehension including quizzes, assignments, and project guidelines
- **Fields**:
  - id: Unique identifier for the assessment
  - title: Descriptive title of the assessment
  - type: Enum (Quiz, Assignment, Project, Practical)
  - description: Overview of the assessment purpose
  - questions: Array of questions for the assessment
  - module: Reference to the parent Course Module
  - week: Reference to the associated Weekly Breakdown
  - maxScore: Maximum possible score
  - timeLimit: Time limit for the assessment (in minutes, null if untimed)
  - passingScore: Minimum score required to pass
- **Relationships**:
  - Belongs to one Course Module
  - Associated with one Weekly Breakdown
- **Validation Rules**:
  - Title must not be empty
  - Type must be one of the defined enum values
  - Questions array must not be empty
  - Max score must be a positive number
  - Passing score must be less than or equal to max score

### Hardware Setup Guide
- **Description**: Detailed instructions for configuring required equipment including RTX workstations, RealSense cameras, Jetson platforms, etc.
- **Fields**:
  - id: Unique identifier for the hardware setup guide
  - title: Descriptive title of the hardware
  - equipmentType: Enum (Workstation, Camera, Robot, Controller, Other)
  - requiredComponents: Array of specific components needed
  - setupSteps: Array of ordered instructions for setup
  - troubleshootingTips: Array of common issues and solutions
  - compatibilityNotes: Additional notes about compatibility
  - estimatedSetupTime: Estimated time to complete setup (in minutes)
- **Relationships**:
  - Referenced by multiple LabExercise entities
- **Validation Rules**:
  - Title must not be empty
  - Setup steps array must not be empty
  - Equipment type must be one of the defined enum values
  - Estimated setup time must be a positive number

## Content Entities

### Textbook Content
- **Description**: Represents individual content sections within the textbook
- **Fields**:
  - id: Unique identifier for the content section
  - title: Title of the content section
  - slug: URL-friendly identifier
  - contentPath: File path to the content (Markdown format)
  - module: Reference to the parent Course Module
  - week: Reference to the associated Weekly Breakdown
  - contentType: Enum (Lecture, Reading, Guide, Reference)
  - isDownloadable: Boolean indicating if content is available for download
  - requiresHighPerformance: Boolean indicating if high-performance machine required
- **Relationships**:
  - Belongs to one Course Module
  - Associated with one Weekly Breakdown (optional)
- **Validation Rules**:
  - Title and slug must not be empty
  - Content path must be valid and exist
  - Content type must be one of the defined enum values

## User Interaction Entities

### Student Progress
- **Description**: Tracks student progress through the textbook content
- **Fields**:
  - id: Unique identifier for the progress record
  - studentId: Identifier for the student
  - contentId: Reference to the content being tracked
  - completionStatus: Enum (NotStarted, InProgress, Completed)
  - completionDate: Date when content was completed
  - assessmentScore: Score from associated assessment (if applicable)
  - timeSpent: Time spent on the content (in minutes)
- **Relationships**:
  - Associated with one Textbook Content
- **Validation Rules**:
  - Student ID must not be empty
  - Content ID must reference a valid content item
  - Completion status must be one of the defined enum values
  - Assessment score (if provided) must be between 0 and maxScore of associated assessment

### User Note
- **Description**: Allows students to add personal notes to textbook content
- **Fields**:
  - id: Unique identifier for the note
  - studentId: Identifier for the student who created the note
  - contentId: Reference to the content section
  - noteText: The actual note content
  - createdAt: Timestamp when the note was created
  - updatedAt: Timestamp when the note was last updated
- **Relationships**:
  - Associated with one Textbook Content
- **Validation Rules**:
  - Student ID must not be empty
  - Content ID must reference a valid content item
  - Note text must not be empty and have reasonable length limits