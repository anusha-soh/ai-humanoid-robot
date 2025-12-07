---
id: 008
title: UI Theme Implementation Plan
stage: plan
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: minimal-ui-theme
branch: 002-minimal-ui-theme
user: system
command: /sp.plan
labels: ["planning", "ui", "architecture", "design-decisions", "implementation"]
links:
  spec: specs/002-minimal-ui-theme/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-minimal-ui-theme/plan.md
tests:
  - Constitution compliance check (PASSED)
  - Technical context validation
  - ADR quality assessment
---

## Prompt

/sp.plan

## Response snapshot

Created comprehensive implementation plan for minimal sophisticated UI theme with 5 architectural decision records and detailed implementation phases.

**Technical Context Defined:**
- Platform: Docusaurus 3.9.2 static site generator
- Stack: TypeScript 5.6.2, React 19.0.0, Node.js 20+
- Testing: pa11y (WCAG), WebAIM (contrast), Chrome DevTools (responsive)
- Constraints: Build <2min, WCAG 2.1 AA, no content modification
- Scope: 21 pages, 2 themes (light/dark), 4 SVG icons, ~400 lines CSS

**5 Key Architectural Decisions (ADRs):**

1. **ADR-001: CSS Custom Properties vs Styled Components**
   - Decision: Use CSS variables (Infima-native)
   - Rationale: Zero runtime overhead, native dark mode support, fastest build
   - Trade-off: Less type safety but optimal performance

2. **ADR-002: Component Swizzling Strategy**
   - Decision: Wrap swizzling (safe mode) over eject
   - Rationale: Preserves Docusaurus upgrade compatibility
   - Trade-off: Less control but minimal maintenance

3. **ADR-003: Icon Asset Format**
   - Decision: Inline SVG in React components
   - Rationale: Dynamic styling, no HTTP requests, full accessibility control
   - Trade-off: Slightly larger bundle but best performance for 4 icons

4. **ADR-004: Typography Stack**
   - Decision: System font stack (no web fonts)
   - Rationale: Instant load (0ms), native OS feel, zero network cost
   - Fonts: SF/Segoe UI (body), SF Mono/Consolas (code)

5. **ADR-005: Responsive Breakpoints**
   - Decision: Docusaurus default breakpoints, mobile-first CSS
   - Rationale: Consistent with ecosystem, well-tested, better performance
   - Breakpoints: 576px, 768px, 996px, 1280px

**Implementation Phases (20-28 hours):**
- Phase 0: Research & Design Tokens (3-4h)
- Phase 1: Component Specs & Swizzling (5-6h)
- Phase 2: CSS Implementation (4-6h)
- Phase 3: React Components (4-5h)
- Phase 4: Testing & Validation (5-7h)
- Phase 5: Deployment & Documentation (1-2h)

**Testing Strategy:**
- 7 test types: Contrast, Responsive, Dark Mode, Typography, Accessibility, Build Time, Comprehension
- Tools: WebAIM, pa11y, Chrome DevTools, Hemingway Editor
- Acceptance: All 8 success criteria validated

**Constitution Compliance:** ✅ PASSED
- Build success + <2min
- WCAG 2.1 AA compliance
- Asset optimization (<500KB images)
- Docusaurus conventions
- No content changes

**Project Structure:**
- Source: `src/css/`, `src/components/`, `src/pages/`
- Assets: `static/img/icons/` (4 SVG module icons)
- Docs: `specs/002-minimal-ui-theme/` (plan, research, design tokens, component specs)

## Outcome

- ✅ Impact: Complete architectural plan ready for implementation with 5 ADRs and detailed phase breakdown
- 🧪 Tests: Constitution check PASSED (8/8 requirements); Testing strategy defined for 7 test types
- 📁 Files: Created plan.md (~800 lines) with technical context, ADRs, phases, testing, risks
- 🔁 Next prompts: Begin Phase 0 (research design systems, define tokens); /sp.tasks not needed (plan includes implementation steps)
- 🧠 Reflection: Plan successfully balances technical rigor (5 ADRs with trade-offs) with practical implementation guidance (5 phases with time estimates). CSS variables + wrap swizzling approach minimizes risk while maximizing Docusaurus compatibility.

## Evaluation notes (flywheel)

- Failure modes observed: None - planning completed successfully with all ADRs documented
- Graders run and results (PASS/FAIL): Constitution check PASSED (8/8); ADR quality PASSED (all have Context, Options, Rationale, Trade-offs, Validation)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Consider visual mockups for Phase 1 to accelerate component design; evaluate color palette generators for Phase 0 token definition
