# Implementation Leak Fixes - Physical AI Book Specification

**Date**: 2025-12-05
**Spec File**: `specs/001-physical-ai-book/spec.md`
**Status**: ✅ All implementation leaks removed

---

## Summary

Removed **9 implementation leaks** from the specification by replacing specific build tools, file paths, and platform names with generic, outcome-focused language.

**Principle**: Specifications should describe **WHAT** (outcomes, requirements) not **HOW** (tools, file paths, specific technologies for building).

---

## Changes Made

### 1. **FR-001** - Removed "Docusaurus" reference

**Before**:
```markdown
Book MUST contain exactly 4 modules with 4 chapters each (16 chapters total)
following the Docusaurus modular structure defined in the constitution.
```

**After**:
```markdown
Book MUST contain exactly 4 modules with 4 chapters each (16 chapters total)
following the modular structure defined in the constitution.
```

**Why**: "Docusaurus" is a build tool choice, not a content requirement.

---

### 2. **FR-023** - Removed "npm run build" command

**Before**:
```markdown
Book MUST build cleanly with `npm run build` with zero errors per constitution Section VI.
```

**After**:
```markdown
Book MUST build successfully through automated build process with zero errors
per constitution Section VI.
```

**Why**: Specific build commands are implementation details.

---

### 3. **FR-025** - Removed "static/img/" file path

**Before**:
```markdown
All images MUST be PNG or SVG format, maximum 500KB per file,
stored in `static/img/` organized by module.
```

**After**:
```markdown
All images MUST be PNG or SVG format, maximum 500KB per file,
stored in designated asset directories organized by module per constitution.
```

**Why**: File paths are implementation structure, not content requirements.

---

### 4. **FR-026** - Removed specific file paths

**Before**:
```markdown
File structure MUST follow: `docs/module-01/chapter-01.md` through
`docs/module-04/chapter-04.md` per constitution.
```

**After**:
```markdown
Content MUST be organized hierarchically with 4 modules (numbered 01-04)
each containing 4 chapters (numbered 01-04) per constitution file structure standards.
```

**Why**: Exact file paths are implementation details; organizational structure is the requirement.

---

### 5. **FR-028** - Removed "GitHub Pages" platform name

**Before**:
```markdown
Deployment to GitHub Pages MUST succeed without manual intervention.
```

**After**:
```markdown
Deployment to hosting platform MUST succeed automatically without manual intervention.
```

**Why**: Specific hosting platform is implementation choice, not content requirement.

---

### 6. **Asset Entity** - Removed file paths and naming conventions

**Before**:
```markdown
Asset: Images, diagrams, or other media. Stored in `static/img/module-XX/`
with naming convention `module-XX-chapter-YY-description.ext`. PNG/SVG format, max 500KB.
```

**After**:
```markdown
Asset: Images, diagrams, or other media. Organized by module and chapter
per constitution asset management standards. PNG/SVG format, max 500KB per file.
```

**Why**: File naming conventions and paths are implementation details.

---

### 7. **SC-009** - Removed "npm run build" command

**Before**:
```markdown
`npm run build` completes with zero errors and zero warnings in under 2 minutes.
```

**After**:
```markdown
Automated build process completes with zero errors and zero warnings in under 2 minutes.
```

**Why**: Build commands are implementation details.

---

### 8. **SC-012** - Removed "GitHub Pages" platform name

**Before**:
```markdown
GitHub Pages deployment succeeds automatically without manual intervention.
```

**After**:
```markdown
Deployment to hosting platform succeeds automatically without manual intervention.
```

**Why**: Specific hosting platform is implementation choice.

---

### 9. **Dependencies Section** - Added clarification and restructured

**Before**: Mixed content and infrastructure dependencies without distinction.

**After**: Organized into three subsections:

```markdown
## Dependencies

> **Note**: Dependencies are divided into two categories:
> - **Content Dependencies**: Technologies the book teaches (ROS 2, Gazebo, Isaac, etc.)
> - **Infrastructure Dependencies**: Tools for building and hosting the book (specified in constitution)

### Content Dependencies
1. External Documentation (ROS 2, Gazebo, NVIDIA Isaac, OpenAI)
2. Software Ecosystem Stability (ROS 2 Humble LTS, Gazebo, Isaac Sim)

### Infrastructure Dependencies
3. Information Verification Tool (MCP Server contacx7)
4. Build Platform (Static site generator per constitution)
5. Hosting Platform (Static hosting service per constitution)

### Content & Infrastructure Dependencies
6-10. [Mixed dependencies]
```

**Why**: Clarifies which dependencies are about content (what readers learn) vs infrastructure (how book is built).

---

## What Was Kept (NOT Implementation Leaks)

These technologies remain in the spec because they are **CONTENT** (what the book teaches):

✅ **ROS 2** - Subject matter (readers learn ROS 2)
✅ **Gazebo / Isaac Sim** - Subject matter (readers learn simulation)
✅ **Python, PyTorch, OpenCV** - Subject matter (readers learn these tools)
✅ **Ubuntu 22.04** - Reader environment requirement
✅ **URDF, VSLAM, Nav2** - Subject matter (core concepts taught)
✅ **OpenAI Whisper, LLMs** - Subject matter (Module 4 content)

**Principle**: If it's **what readers build/learn**, it stays. If it's **how the book is built**, it's removed or genericized.

---

## Verification Checklist

- [x] No build commands (npm, yarn, etc.)
- [x] No specific file paths (docs/, static/img/, etc.)
- [x] No platform names in requirements (Docusaurus, GitHub Pages)
- [x] No file naming conventions in requirements
- [x] Dependencies clarified (content vs infrastructure)
- [x] All requirements focus on outcomes, not tools
- [x] Constitution references preserved (source of authority)

---

## Impact Assessment

| Category | Before | After | Impact |
|----------|--------|-------|--------|
| **Implementation Leaks** | 9 found | 0 remaining | High - Spec now properly technology-agnostic |
| **Non-Goals** | 10 defined | 10 defined (unchanged) | None - Already clear |
| **Content Requirements** | Preserved | Preserved | None - ROS 2, Gazebo, etc. still mentioned as content |
| **Readability** | Good | Better | Dependencies now clearly categorized |

---

## Next Steps

1. ✅ **Specification is now clean** - Ready for `/sp.plan`
2. **Constitution contains implementation details** - This is correct! Constitution specifies:
   - Docusaurus 3.x
   - Node.js 18.x+
   - GitHub Pages
   - MCP server contacx7
   - File structure (`docs/module-01/`, `static/img/`)
3. **Plan will reference constitution** - Implementation decisions will cite constitution as authority

---

## Key Takeaway

**Specifications** = What the product must do (outcomes, user value)
**Constitution** = How the project is constrained (tools, standards, processes)
**Plan** = How we implement (architecture, decisions, rationale)

Your spec now correctly focuses on **WHAT** without leaking **HOW**.
