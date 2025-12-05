---
id: 002
title: Improve constitution testability and coverage
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: none
labels: ["constitution", "testability", "standards", "plagiarism", "review-process"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - none
---

## Prompt

Review my Constitution at .specify/memory/constitution.md and improve it:

1. Are all standards testable (not vague)?
   - ❌ Vague: "Papers should be well-written"
   - ✅ Testable: "Flesch-Kincaid grade 10-12; active voice 75%+ of time"

2. Did I cover essential categories?
   - Citation accuracy
   - Source verification
   - Writing clarity
   - Plagiarism checking
   - Review process

3. Are any standards unrealistic?

Suggest 2-3 concrete improvements.

## Response snapshot

Implemented three concrete improvements to make constitution standards measurable and complete:

**1. Measurable Writing Clarity Standards (Section V, lines 36-43)**
- Added Flesch-Kincaid grade level 10-12 requirement
- Set max 25 words/sentence average
- Required 75%+ active voice usage
- Specified 1 diagram/table per 500 words minimum
- Added difficulty labels requirement for complex concepts

**2. New Plagiarism & Originality Section (Section VII, lines 53-61)**
- Added plagiarism check requirement: < 15% similarity excluding quotes
- Required code attribution with source URL, license verification, and modification notes
- Established quote policy for >2 sentences
- Added AI-generated content review requirement
- Included self-plagiarism disclosure policy

**3. Concrete Review Process Criteria (lines 136-149)**
- Technical accuracy: 2+ reviewers, zero factual errors, all code runs
- Style consistency: checklist-based with auto-linter checks
- Automated checks with numeric thresholds:
  - npm build: zero errors
  - Link checker: zero internal 404s, <5% external broken
  - Readability: Flesch-Kincaid 10-12
  - Image size: all <500KB
- WCAG 2.1 AA compliance requirement

## Outcome

- ✅ Impact: Constitution now has enforceable, measurable quality gates instead of subjective standards
- 🧪 Tests: No automated tests; manual verification of constitution completeness
- 📁 Files: Modified .specify/memory/constitution.md (3 sections enhanced/added)
- 🔁 Next prompts: Consider adding glossary/linter configuration files to enforce terminology consistency
- 🧠 Reflection: Transformation from aspirational principles to testable acceptance criteria strengthens project governance

## Evaluation notes (flywheel)

- Failure modes observed: Original constitution had unmeasurable standards ("concise", "regular", "uniform") that couldn't be enforced
- Graders run and results (PASS/FAIL): Manual review PASS - all three improvements add quantifiable metrics
- Prompt variant (if applicable): none
- Next experiment (smallest change to try): Implement automated constitution compliance checker (linter) to validate content against new standards
