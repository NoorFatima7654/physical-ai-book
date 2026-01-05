---
name: failed-task-generation
description: Failed task generation due to missing feature directory
model: claude-sonnet
---

## Prompt

Break down the plan into specific executable tasks as a checklist. Include:
1. Docusaurus setup tasks
2. Chapter development tasks (for 1 chapter with 3 lessons)

## Response snapshot

I couldn't generate the tasks because the feature directory E:\Q4-Hackathon\specs\1-physical-ai-book-spec was not found. You need to run /sp.specify first to create the feature structure.

## Outcome

- ✅ Impact: Failed to generate tasks
- 🧪 Tests: N/A
- 📁 Files: N/A
- 🔁 Next prompts: Run /sp.specify
- 🧠 Reflection: User attempted to generate tasks without a specified feature directory, leading to a prerequisite failure. Directed user to /sp.specify.

## Evaluation notes (flywheel)

- Failure modes observed: Missing feature directory for task generation
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure proper error handling and user guidance for missing prerequisites in skill invocations.
