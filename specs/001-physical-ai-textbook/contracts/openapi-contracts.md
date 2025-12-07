# API Contracts: Physical AI & Humanoid Robotics Textbook

## Overview
This document defines the API contracts for the Physical AI & Humanoid Robotics textbook application. These APIs support interactive features, student progress tracking, and content management.

## Base URL
```
https://physical-ai-textbook.example.com/api/v1
```

## Content Management API

### Get All Course Modules
```
GET /modules
```
**Description**: Retrieve all course modules in the textbook.

**Response**:
```json
{
  "data": [
    {
      "id": "ros2",
      "title": "ROS 2",
      "description": "Robotic operating system fundamentals",
      "learningObjectives": [
        "Understand ROS 2 architecture",
        "Create and manage nodes"
      ],
      "contentPath": "/docs/ros2",
      "order": 1
    },
    {
      "id": "gazebo-unity",
      "title": "Gazebo/Unity",
      "description": "Simulation environments for robotics",
      "learningObjectives": [
        "Create simulation environments",
        "Implement physics modeling"
      ],
      "contentPath": "/docs/gazebo-unity",
      "order": 2
    }
  ],
  "meta": {
    "total": 4
  }
}
```

### Get Module by ID
```
GET /modules/{moduleId}
```
**Description**: Retrieve a specific course module.

**Path Parameters**:
- moduleId: Unique identifier for the module (ros2, gazebo-unity, nvidia-isaac, vla)

**Response**:
```json
{
  "data": {
    "id": "ros2",
    "title": "ROS 2",
    "description": "Robotic operating system fundamentals",
    "learningObjectives": [
      "Understand ROS 2 architecture",
      "Create and manage nodes"
    ],
    "contentPath": "/docs/ros2",
    "order": 1,
    "weeklyBreakdowns": [
      {
        "id": "week1-2",
        "weekNumber": 1,
        "title": "Introduction to ROS 2",
        "topics": ["Architecture", "Nodes", "Topics"],
        "estimatedHours": 6
      }
    ],
    "labExercises": [
      {
        "id": "lab-ros2-001",
        "title": "Creating Your First ROS 2 Node",
        "description": "Basic node creation and communication",
        "difficulty": "Beginner",
        "estimatedDuration": 90
      }
    ],
    "assessments": [
      {
        "id": "quiz-ros2-001",
        "title": "ROS 2 Fundamentals Quiz",
        "type": "Quiz",
        "maxScore": 100,
        "passingScore": 70
      }
    ]
  }
}
```

## Progress Tracking API

### Track Content Progress
```
POST /progress
```
**Description**: Track a student's progress through content.

**Request Body**:
```json
{
  "studentId": "student-12345",
  "contentId": "ros2-week1-lecture",
  "completionStatus": "Completed",
  "timeSpent": 45
}
```

**Response**:
```json
{
  "data": {
    "id": "progress-123",
    "studentId": "student-12345",
    "contentId": "ros2-week1-lecture",
    "completionStatus": "Completed",
    "completionDate": "2025-09-15T10:30:00Z",
    "timeSpent": 45
  }
}
```

### Get Student Progress
```
GET /progress?studentId={studentId}&moduleId={moduleId}
```
**Description**: Retrieve a student's progress in a specific module.

**Query Parameters**:
- studentId: Unique identifier for the student
- moduleId: Unique identifier for the module (optional)

**Response**:
```json
{
  "data": [
    {
      "id": "progress-123",
      "studentId": "student-12345",
      "contentId": "ros2-week1-lecture",
      "completionStatus": "Completed",
      "completionDate": "2025-09-15T10:30:00Z",
      "timeSpent": 45
    }
  ]
}
```

## Assessment API

### Submit Assessment
```
POST /assessments/{assessmentId}/submit
```
**Description**: Submit answers for an assessment.

**Path Parameters**:
- assessmentId: Unique identifier for the assessment

**Request Body**:
```json
{
  "studentId": "student-12345",
  "answers": [
    {
      "questionId": "q1",
      "answer": "optionA"
    },
    {
      "questionId": "q2",
      "answer": "free text response"
    }
  ]
}
```

**Response**:
```json
{
  "data": {
    "assessmentId": "quiz-ros2-001",
    "studentId": "student-12345",
    "score": 85,
    "maxScore": 100,
    "passingScore": 70,
    "isPassing": true,
    "submittedAt": "2025-09-15T11:00:00Z"
  }
}
```

### Get Assessment Results
```
GET /assessments/{assessmentId}/results?studentId={studentId}
```
**Description**: Retrieve results for a specific assessment for a student.

**Path Parameters**:
- assessmentId: Unique identifier for the assessment

**Query Parameters**:
- studentId: Unique identifier for the student

**Response**:
```json
{
  "data": {
    "assessmentId": "quiz-ros2-001",
    "studentId": "student-12345",
    "score": 85,
    "maxScore": 100,
    "passingScore": 70,
    "isPassing": true,
    "submittedAt": "2025-09-15T11:00:00Z",
    "breakdown": [
      {
        "questionId": "q1",
        "pointsAwarded": 5,
        "maxPoints": 5
      }
    ]
  }
}
```

## Content Search API

### Search Content
```
GET /search?q={query}
```
**Description**: Search across all textbook content.

**Query Parameters**:
- q: Search query string
- limit: Number of results to return (default: 10, max: 50)
- page: Page number for pagination (default: 1)

**Response**:
```json
{
  "data": [
    {
      "id": "ros2-week1-lecture",
      "title": "Introduction to ROS 2 Architecture",
      "contentPath": "/docs/ros2/week1-2",
      "module": "ROS 2",
      "type": "Lecture",
      "relevance": 0.95,
      "highlight": "ROS 2 introduces a new architecture that provides better <mark>performance</mark> and <mark>reliability</mark>..."
    }
  ],
  "meta": {
    "total": 1,
    "page": 1,
    "limit": 10
  }
}
```

## User Note API

### Create Note
```
POST /notes
```
**Description**: Create a personal note for textbook content.

**Request Body**:
```json
{
  "studentId": "student-12345",
  "contentId": "ros2-week1-lecture",
  "noteText": "Key concept: DDS provides reliable messaging between nodes"
}
```

**Response**:
```json
{
  "data": {
    "id": "note-67890",
    "studentId": "student-12345",
    "contentId": "ros2-week1-lecture",
    "noteText": "Key concept: DDS provides reliable messaging between nodes",
    "createdAt": "2025-09-15T10:45:00Z",
    "updatedAt": "2025-09-15T10:45:00Z"
  }
}
```

### Get Notes for Content
```
GET /notes?studentId={studentId}&contentId={contentId}
```
**Description**: Retrieve all notes for a student on specific content.

**Query Parameters**:
- studentId: Unique identifier for the student
- contentId: Unique identifier for the content

**Response**:
```json
{
  "data": [
    {
      "id": "note-67890",
      "studentId": "student-12345",
      "contentId": "ros2-week1-lecture",
      "noteText": "Key concept: DDS provides reliable messaging between nodes",
      "createdAt": "2025-09-15T10:45:00Z",
      "updatedAt": "2025-09-15T10:45:00Z"
    }
  ]
}
```

## Error Responses

All API endpoints follow this standard error response format:

```json
{
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "The request data is invalid",
    "details": [
      {
        "field": "studentId",
        "message": "This field is required"
      }
    ]
  }
}
```

### Common Error Codes
- `VALIDATION_ERROR`: Request data didn't pass validation
- `NOT_FOUND`: Requested resource doesn't exist
- `UNAUTHORIZED`: User doesn't have permissions for the requested action
- `RATE_LIMITED`: Too many requests in a given time period
- `INTERNAL_ERROR`: An unexpected server error occurred