# Tasks: Minimal Sophisticated UI Theme

**Input**: Design documents from `/specs/002-minimal-ui-theme/`
**Prerequisites**: plan.md, spec.md (3 user stories: P1, P2, P3)

**Tests**: Not requested in specification - tasks focus on implementation only.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

All paths are relative to repository root:
- `src/css/` - CSS customization
- `src/components/` - React components
- `src/pages/` - Custom pages
- `static/img/icons/` - SVG icon assets
- `docusaurus.config.ts` - Docusaurus configuration

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing Docusaurus setup and prepare for theme customization

- [ ] T001 Verify Docusaurus 3.9.2 installation and dependencies in package.json
- [ ] T002 Create src/css/ directory if not exists for custom theme CSS
- [ ] T003 Create src/components/ directory if not exists for React components
- [ ] T004 Create static/img/icons/ directory for SVG module icons

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core design system and theming infrastructure that ALL user stories depend on

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [ ] T005 Research minimal UI design systems (GitHub Docs, Stripe Docs, Tailwind, Linear) and document findings
- [ ] T006 [P] Define color tokens for light mode in specs/002-minimal-ui-theme/design-tokens.md (neutral grays + blue accent)
- [ ] T007 [P] Define color tokens for dark mode in specs/002-minimal-ui-theme/design-tokens.md
- [ ] T008 [P] Validate all color combinations pass 4.5:1 contrast ratio using WebAIM Contrast Checker
- [ ] T009 [P] Define typography tokens (16px body, system fonts, 14px code) in specs/002-minimal-ui-theme/design-tokens.md
- [ ] T010 [P] Define spacing scale (4px base unit: 8, 12, 16, 24, 32, 48, 64px) in specs/002-minimal-ui-theme/design-tokens.md
- [ ] T011 Implement base CSS variables in src/css/custom.css for light mode colors
- [ ] T012 Implement CSS variables in src/css/custom.css for dark mode colors using [data-theme='dark'] selector
- [ ] T013 [P] Implement typography CSS variables in src/css/custom.css (system font stack, sizes, line heights)
- [ ] T014 [P] Implement spacing CSS variables in src/css/custom.css
- [ ] T015 Configure docusaurus.config.ts to respect system preference (colorMode.respectPrefersColorScheme: true)
- [ ] T016 Test dark mode toggle for smooth transition without FOUC (SC-005)
- [ ] T017 Run build test to verify <2min build time (constitution requirement)

**Checkpoint**: Foundation ready - theme system established, user story implementation can now begin

---

## Phase 3: User Story 1 - First-Time Visitor Engagement (Priority: P1) 🎯 MVP

**Goal**: Create compelling landing page with hero section and 4 module cards that enables 10-second comprehension

**Independent Test**: Navigate to localhost:3000 and within 10 seconds answer: "What is this book about?", "Who is it for?", and "What will I learn?"

**Success Criteria**: SC-001 (10-second comprehension), SC-002 (4 modules in first scroll), SC-008 (above fold at 1920x1080)

### SVG Icon Creation for US1

- [ ] T018 [P] [US1] Create module-01-nervous-system.svg icon (ROS 2 network/nodes, 48x48, 2px stroke, outline) in static/img/icons/
- [ ] T019 [P] [US1] Create module-02-digital-twin.svg icon (Gazebo cube/3D, 48x48, 2px stroke, outline) in static/img/icons/
- [ ] T020 [P] [US1] Create module-03-ai-brain.svg icon (Isaac Sim camera/eye, 48x48, 2px stroke, outline) in static/img/icons/
- [ ] T021 [P] [US1] Create module-04-vla.svg icon (VLA brain/chat bubble, 48x48, 2px stroke, outline) in static/img/icons/

### Hero Component for US1

- [ ] T022 [US1] Create src/components/Hero/index.tsx with book title, tagline, description, and "Get Started" CTA button
- [ ] T023 [US1] Create src/components/Hero/styles.module.css with hero styles (centered text, gradient/solid background, responsive sizing)
- [ ] T024 [US1] Implement responsive typography in Hero component (2rem mobile → 4rem desktop for title)
- [ ] T025 [US1] Add Link to /intro for "Get Started" button in Hero component

### Module Cards Component for US1

- [ ] T026 [US1] Create src/components/HomepageFeatures/index.tsx for 4 module cards component
- [ ] T027 [US1] Create src/components/HomepageFeatures/styles.module.css with card grid layout (2x2 mobile, 4x1 desktop)
- [ ] T028 [US1] Implement inline SVG icons in HomepageFeatures component using import statements for 4 module icons
- [ ] T029 [US1] Add module card content (icon, title, 2-3 sentence description) for Module 1: Robotic Nervous System
- [ ] T030 [US1] Add module card content for Module 2: Digital Twin
- [ ] T031 [US1] Add module card content for Module 3: AI-Robot Brain
- [ ] T032 [US1] Add module card content for Module 4: Vision-Language-Action
- [ ] T033 [US1] Implement subtle hover effect on module cards (lift + shadow)
- [ ] T034 [US1] Add aria-labels to all SVG icons for accessibility (WCAG 2.1 AA)

### Landing Page Composition for US1

- [ ] T035 [US1] Update src/pages/index.tsx to import Hero and HomepageFeatures components
- [ ] T036 [US1] Compose landing page layout with Hero section followed by HomepageFeatures in main element
- [ ] T037 [US1] Add "Who this is for" section to src/pages/index.tsx with prerequisites and target audience
- [ ] T038 [US1] Add "What you'll build" section to src/pages/index.tsx highlighting capstone project (autonomous butler)

### Validation for US1

- [ ] T039 [US1] Test landing page at 375px viewport for no horizontal scroll (SC-004)
- [ ] T040 [US1] Test landing page at 1920x1080 for hero + 4 modules above fold (SC-008)
- [ ] T041 [US1] Conduct 10-second comprehension test with 3-5 test users and verify ≥80% pass rate (SC-001)
- [ ] T042 [US1] Verify all 4 modules visible within first scroll without multiple page scrolls (SC-002)
- [ ] T043 [US1] Verify CTA button navigates to /intro correctly (FR-005)

**Checkpoint**: Landing page complete and independently testable - delivers MVP value (P1 user story)

---

## Phase 4: User Story 2 - Reader Navigation Experience (Priority: P2)

**Goal**: Implement clean, distraction-free reading environment with comfortable typography and intuitive navigation

**Independent Test**: Read any chapter (e.g., /intro or /module-01/chapter-01) and experience consistent typography with clear navigation controls

**Success Criteria**: SC-003 (4.5:1 contrast), SC-006 (2-click navigation), FR-007 (16-18px body), FR-011-FR-015 (navigation)

### Typography Enhancement for US2

- [ ] T044 [P] [US2] Enhance body text CSS in src/css/custom.css (16-18px, 1.6 line height, system font stack)
- [ ] T045 [P] [US2] Enhance heading hierarchy CSS in src/css/custom.css (sizes, weights, spacing, letter-spacing -0.02em)
- [ ] T046 [P] [US2] Enhance code block CSS in src/css/custom.css (14px, monospace font stack: SF Mono, Consolas, Liberation Mono)
- [ ] T047 [P] [US2] Set max line length for readability (60-80 characters per line)
- [ ] T048 [US2] Test typography at 200% browser zoom for readability

### Navigation Styling for US2

- [ ] T049 [P] [US2] Style sidebar navigation in src/css/custom.css for current location indicator (FR-011)
- [ ] T050 [P] [US2] Style previous/next chapter controls in src/css/custom.css (FR-012)
- [ ] T051 [P] [US2] Implement collapsible navigation menu CSS for mobile devices <768px (FR-013)
- [ ] T052 [P] [US2] Style interactive element hover/active states in src/css/custom.css (buttons, links) (FR-015)

### Content Presentation Styling for US2

- [ ] T053 [P] [US2] Ensure code syntax highlighting colors harmonize with theme (max 6 colors) in src/css/custom.css (SC-007, FR-016)
- [ ] T054 [P] [US2] Style images and diagrams for appropriate sizing and spacing in src/css/custom.css (FR-018)
- [ ] T055 [P] [US2] Style lists and tables with clear visual formatting in src/css/custom.css (FR-019)

### Responsive Layout for US2

- [ ] T056 [US2] Implement mobile-first responsive CSS with Docusaurus breakpoints (576px, 768px, 996px, 1280px) in src/css/custom.css
- [ ] T057 [US2] Add media query for 768px+ (tablet): wider containers, 2-column layouts where appropriate
- [ ] T058 [US2] Add media query for 996px+ (desktop): 3-column layouts, side-by-side content + sidebar, full navigation
- [ ] T059 [US2] Add media query for 1280px+ (extra large): max-width containers, larger spacing

### Validation for US2

- [ ] T060 [US2] Test responsive design at 375px, 768px, 1024px, 1920px viewports using Chrome DevTools (SC-004, FR-014)
- [ ] T061 [US2] Verify no horizontal scroll at 375px on chapter pages (SC-004)
- [ ] T062 [US2] Test keyboard navigation (Tab, Enter, Esc) for accessibility
- [ ] T063 [US2] Test navigation from chapter 1 to chapter 16 in maximum 2 clicks (SC-006)
- [ ] T064 [US2] Verify dark mode colors remain harmonious and readable when toggled on chapter pages
- [ ] T065 [US2] Run pa11y accessibility scan on 3 chapter pages to verify WCAG 2.1 AA compliance (SC-003)

**Checkpoint**: Reading experience complete - typography, navigation, and responsiveness functional

---

## Phase 5: User Story 3 - Visual Brand Identity (Priority: P3)

**Goal**: Establish cohesive visual identity with consistent colors, spacing, and visual elements across all page types

**Independent Test**: Navigate between chapters, appendix, and landing page to observe consistent color palette, typography, and spacing

**Success Criteria**: FR-010 (visual consistency), FR-006 (color palette), FR-008 (whitespace)

### Visual Consistency for US3

- [ ] T066 [P] [US3] Apply consistent whitespace/spacing system to all page types in src/css/custom.css (FR-008)
- [ ] T067 [P] [US3] Ensure neutral gray + blue accent palette consistent across all components in src/css/custom.css (FR-006)
- [ ] T068 [P] [US3] Style 404 page to match theme in src/css/custom.css
- [ ] T069 [P] [US3] Style appendix pages (prerequisites, glossary, troubleshooting, resources) to match theme

### Theme Refinement for US3

- [ ] T070 [US3] Add subtle transitions for interactive elements (buttons, links) in src/css/custom.css (150-200ms)
- [ ] T071 [US3] Refine focus indicators for accessibility (visible, high contrast) in src/css/custom.css
- [ ] T072 [US3] Add custom scrollbar styling for consistent branding in src/css/custom.css (optional, browser-dependent)

### Validation for US3

- [ ] T073 [US3] Visual test: Navigate landing → chapter → appendix and verify consistent colors
- [ ] T074 [US3] Visual test: Navigate landing → chapter → appendix and verify consistent typography
- [ ] T075 [US3] Visual test: Navigate landing → chapter → appendix and verify consistent spacing
- [ ] T076 [US3] Verify code examples on chapter pages integrate harmoniously with theme colors
- [ ] T077 [US3] Verify diagrams/images complement minimal aesthetic (no clashing colors)

**Checkpoint**: Visual brand identity established across all page types

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final testing, optimization, and quality gates before deployment

### Build & Performance Testing

- [ ] T078 Run full build test with `npm run build` and verify completes in <120 seconds (constitution requirement)
- [ ] T079 Measure incremental build time (single CSS file change) for development efficiency
- [ ] T080 Check output bundle size for any regressions

### Accessibility Testing

- [ ] T081 Run pa11y on landing page to verify WCAG 2.1 AA compliance (0 violations)
- [ ] T082 Run pa11y on 3 sample chapter pages to verify WCAG 2.1 AA compliance
- [ ] T083 Test screen reader announcements (NVDA or JAWS) on landing page and chapter page
- [ ] T084 Verify all interactive elements have minimum 44x44px touch target for mobile

### Contrast Ratio Validation

- [ ] T085 Test 8 color combinations (light + dark modes) with WebAIM Contrast Checker for 4.5:1 ratio (SC-003)
- [ ] T086 Document contrast test results in specs/002-minimal-ui-theme/test-report.md

### Edge Case Testing

- [ ] T087 Test with custom browser fonts enabled (verify graceful degradation)
- [ ] T088 Test with browser zoom at 200% (verify layout doesn't break)
- [ ] T089 Test with CSS disabled (verify content still accessible)
- [ ] T090 Test long chapter titles in navigation (verify no layout breaking)
- [ ] T091 Test dark mode with conflicting system preferences (verify override works)

### Final Validation

- [ ] T092 Capture screenshots of landing page, chapter page, appendix page in light mode for documentation
- [ ] T093 Capture screenshots of landing page, chapter page, appendix page in dark mode for documentation
- [ ] T094 Create before/after comparison screenshots for PR description
- [ ] T095 Verify all success criteria (SC-001 to SC-008) are met and documented
- [ ] T096 Verify all functional requirements (FR-001 to FR-019) are implemented
- [ ] T097 Update docs/intro.md status banner if present (change from "Under Development" to current phase)

### Deployment Preparation

- [ ] T098 Commit all changes to 002-minimal-ui-theme branch with descriptive commit message
- [ ] T099 Create pull request with title "feat: implement minimal sophisticated UI theme"
- [ ] T100 Add PR description with screenshots, success criteria checklist, and testing summary
- [ ] T101 Deploy preview to GitHub Pages for team review
- [ ] T102 Tag reviewers and request feedback on PR

**Checkpoint**: All quality gates passed, ready for merge and deployment

---

## Dependencies & Parallel Execution

### User Story Dependencies (Completion Order)

```
Phase 1 (Setup) → Phase 2 (Foundational)
                       ↓
                   [Parallel]
                ↙      ↓      ↘
          US1 (P1)  US2 (P2)  US3 (P3)
                ↘      ↓      ↙
                  [Converge]
                       ↓
              Phase 6 (Polish)
```

**Key Insights**:
- Phase 1 & 2 MUST complete sequentially before any user story
- User Stories 1, 2, 3 can be implemented in parallel after Phase 2
- However, recommended order is P1 → P2 → P3 for incremental value delivery
- Phase 6 requires all user stories complete

### Parallel Execution Opportunities

**Within Phase 2 (Foundational)**:
- T006-T010 (token definitions) can run in parallel
- T011-T014 (CSS implementations) can run in parallel after tokens defined

**Within User Story 1**:
- T018-T021 (all 4 SVG icons) can be created in parallel
- T022-T023 (Hero component) parallel with T026-T027 (HomepageFeatures component)
- T029-T032 (module card content) can be added in parallel

**Within User Story 2**:
- T044-T048 (typography) can run in parallel
- T049-T052 (navigation styling) can run in parallel
- T053-T055 (content presentation) can run in parallel

**Within User Story 3**:
- T066-T069 (visual consistency) can be applied in parallel

**Within Phase 6 (Polish)**:
- T092-T093 (screenshots) can be captured in parallel
- T081-T082 (pa11y scans) can run in parallel

---

## Implementation Strategy

### MVP Scope (Minimum Viable Product)

**Recommended MVP**: Complete **only Phase 1 + Phase 2 + User Story 1 (P1)**

This delivers:
- ✅ Enhanced landing page with hero and 4 module cards
- ✅ 10-second comprehension test passes
- ✅ Above-the-fold content on desktop
- ✅ Responsive design (mobile-friendly)
- ✅ Dark mode support
- ✅ Complete, independently testable increment

**Total MVP Tasks**: T001-T043 (43 tasks)

**Estimated Time**: 8-12 hours
- Phase 1: 30 min
- Phase 2: 3-4 hours
- User Story 1: 4-6 hours
- Validation: 1-2 hours

### Incremental Delivery

After MVP, deliver incrementally:

1. **Increment 1** (MVP): US1 only → Landing page functional
2. **Increment 2**: US1 + US2 → Landing page + reading experience
3. **Increment 3**: US1 + US2 + US3 → Complete theme
4. **Final**: All + Polish → Production-ready

Each increment is independently deployable and testable.

---

## Task Summary

**Total Tasks**: 102
**Parallelizable Tasks**: 32 tasks marked with [P]
**User Story Breakdown**:
- Setup: 4 tasks (T001-T004)
- Foundational: 13 tasks (T005-T017)
- User Story 1 (P1): 26 tasks (T018-T043) - **MVP**
- User Story 2 (P2): 22 tasks (T044-T065)
- User Story 3 (P3): 12 tasks (T066-T077)
- Polish: 25 tasks (T078-T102)

**Validation Tasks**: 25 tasks focused on testing and quality assurance
**Success Criteria Mapped**: All 8 success criteria (SC-001 to SC-008) validated in tasks
**Functional Requirements Mapped**: All 19 functional requirements (FR-001 to FR-019) implemented

**Critical Path**: Phase 1 → Phase 2 → US1 → US2 → US3 → Polish (sequential if single developer)
**Optimal Path**: Phase 1 → Phase 2 → [US1 ∥ US2 ∥ US3] → Polish (parallel if multiple developers)

---

**Next Step**: Begin with T001 (verify Docusaurus setup) and proceed sequentially through Phase 1 and Phase 2 before starting User Story 1.
