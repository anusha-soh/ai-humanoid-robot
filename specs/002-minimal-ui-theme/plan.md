# Implementation Plan: Minimal Sophisticated UI Theme

**Branch**: `002-minimal-ui-theme` | **Date**: 2025-12-07 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-minimal-ui-theme/spec.md`

## Summary

Implement a minimal, sophisticated UI theme for the Physical AI & Humanoid Robotics book with enhanced landing page, neutral gray color palette with blue accents, and optimized reader experience. The implementation focuses on CSS customization within Docusaurus framework, creating a custom landing page component, and providing SVG icons for the 4 module cards. All changes are visual-only with no impact on content structure or build pipeline.

**Technical Approach**: Leverage Docusaurus 3.x theming system with custom CSS overrides, React component swizzling for landing page customization, and system preference-aware dark mode implementation.

## Technical Context

**Platform**: Docusaurus 3.9.2 (static site generator)
**Language/Version**: TypeScript 5.6.2, React 19.0.0, Node.js 20+
**Primary Dependencies**:
- `@docusaurus/core`: 3.9.2 (site generation)
- `@docusaurus/preset-classic`: 3.9.2 (default theme)
- `@mdx-js/react`: 3.0.0 (MDX support)
- `prism-react-renderer`: 2.3.0 (code syntax highlighting)

**Storage**: Static file generation (Markdown → HTML)
**Testing**:
- Visual regression testing (manual review)
- Build validation (`npm run build`)
- Accessibility testing (`pa11y` for WCAG 2.1 AA)
- Contrast ratio validation (WebAIM tools)
- Responsive design testing (Chrome DevTools)

**Target Platform**: Modern web browsers (Chrome, Firefox, Safari, Edge - latest 2 versions)
**Project Type**: Static documentation site (Docusaurus)
**Performance Goals**:
- Build time: < 2 minutes (constitution requirement)
- Page load time: < 3 seconds on standard connection
- Image assets: < 500KB each (constitution requirement)
- No flash of unstyled content (FOUC) on theme toggle

**Constraints**:
- Must maintain existing Docusaurus structure (no breaking changes)
- WCAG 2.1 AA compliance (4.5:1 contrast ratio minimum)
- No modification to markdown content files
- Preserve existing navigation structure and functionality
- Build output must remain GitHub Pages compatible

**Scale/Scope**:
- 5 page types: Landing page, chapter pages, appendix pages, module index, 404
- 2 theme modes: Light and dark
- 21 total pages (1 landing + 16 chapters + 4 appendix)
- 4 custom SVG icons for module cards
- 1 custom CSS file with ~300-500 lines

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Mandatory Requirements (from constitution)

- [x] **Build Success**: Must build cleanly with `npm run build` with zero errors (Section VI)
- [x] **Build Time**: < 2 minutes for full build (Section VI)
- [x] **Asset Optimization**: All images < 500KB, PNG/SVG only (Content Standards)
- [x] **No Broken Links**: Internal and external links must be valid (Section VI)
- [x] **Accessibility**: WCAG 2.1 AA compliance (Review Requirements)
- [x] **Docusaurus Conventions**: Must follow Docusaurus best practices (Section III)
- [x] **GitHub Pages Deployment**: Must deploy successfully (Section VI)
- [x] **No Content Changes**: Book content (markdown files) remains untouched (implicit from scope)

### Violations: None

All UI theme changes are purely CSS and component-based, preserving existing content structure and build process. No constitution violations.

## Project Structure

### Documentation (this feature)

```text
specs/002-minimal-ui-theme/
├── plan.md              # This file
├── spec.md              # Feature specification
├── research.md          # Design system research (Phase 0)
├── design-tokens.md     # Color/typography tokens (Phase 1)
├── component-specs.md   # UI component specifications (Phase 1)
└── checklists/
    └── requirements.md  # Spec validation checklist
```

### Source Code (repository root)

```text
src/
├── css/
│   └── custom.css           # Enhanced theme CSS (replaces existing)
├── components/
│   ├── HomepageFeatures/    # Module cards component (swizzled)
│   │   ├── index.tsx
│   │   └── styles.module.css
│   └── Hero/                # Landing page hero section (new)
│       ├── index.tsx
│       └── styles.module.css
└── pages/
    └── index.tsx            # Landing page (enhanced)

static/
└── img/
    ├── icons/               # Module SVG icons (new)
    │   ├── module-01-nervous-system.svg
    │   ├── module-02-digital-twin.svg
    │   ├── module-03-ai-brain.svg
    │   └── module-04-vla.svg
    └── logo.svg             # Site logo (enhanced/replaced)

docusaurus.config.ts         # Theme configuration updates
```

**Structure Decision**: Docusaurus component swizzling approach. Custom CSS in `src/css/custom.css` overrides default theme variables. Landing page uses custom React components with modular CSS. All changes isolated to `src/` directory, preserving existing `docs/` content structure.

## Complexity Tracking

> No constitution violations detected - this section intentionally left empty.

All requirements align with constitution. No complexity justifications needed.

## Key Architectural Decisions

### ADR-001: CSS Custom Properties vs Styled Components

**Decision**: Use Docusaurus CSS custom properties (CSS variables) for theme customization instead of CSS-in-JS libraries.

**Context**:
- Docusaurus uses Infima CSS framework with extensive CSS variable support
- Need to customize colors, typography, spacing across light and dark modes
- Must maintain build time < 2 minutes

**Options Considered**:
1. **CSS Custom Properties (CHOSEN)**: Leverage Infima's existing variable system
2. **Styled Components**: Runtime CSS-in-JS with TypeScript support
3. **Tailwind CSS**: Utility-first CSS framework
4. **SASS/LESS**: CSS preprocessors

**Rationale**:
- CSS variables are Docusaurus-native (no additional dependencies)
- Automatic dark mode support via `[data-theme='dark']` selector
- Zero runtime overhead (pure CSS)
- Minimal build impact (faster than CSS-in-JS)
- Maintainable: Override only changed variables, inherit rest

**Trade-offs**:
- ✅ Fastest build time (no processing overhead)
- ✅ Smallest bundle size (no JS runtime)
- ✅ Native dark mode support
- ✅ Easy to understand for contributors
- ❌ Less type safety than styled components
- ❌ No automatic scoping (must use BEM or modules for components)

**Implementation**:
```css
:root {
  /* Light mode colors */
  --ifm-color-primary: #2563eb;      /* Blue accent */
  --ifm-color-primary-dark: #1d4ed8;
  --ifm-background-color: #ffffff;
  --ifm-font-color-base: #1f2937;    /* Near-black text */
}

[data-theme='dark'] {
  /* Dark mode colors */
  --ifm-color-primary: #60a5fa;      /* Lighter blue for dark bg */
  --ifm-background-color: #111827;
  --ifm-font-color-base: #f9fafb;    /* Near-white text */
}
```

**Validation**: SC-005 (dark mode toggle without page reload), build time < 2min requirement.

---

### ADR-002: Component Swizzling Strategy (Eject vs Wrap)

**Decision**: Use "wrap" (safe) swizzling for HomePage components, avoiding "eject" (unsafe) swizzling.

**Context**:
- Need custom landing page with hero section and module cards
- Docusaurus provides swizzle command to customize theme components
- Two modes: "eject" (full control, breaks on Docusaurus updates) vs "wrap" (safe, wraps existing)

**Options Considered**:
1. **Wrap Swizzling (CHOSEN)**: Enhance existing components with wrappers
2. **Eject Swizzling**: Copy full component source and modify
3. **No Swizzling**: Use only CSS overrides
4. **Custom Plugin**: Build standalone Docusaurus plugin

**Rationale**:
- Wrap swizzling preserves Docusaurus update compatibility
- Landing page (src/pages/index.tsx) doesn't require swizzling (can fully replace)
- HomepageFeatures can be safely swizzled to add module cards
- Minimize maintenance burden on Docusaurus version upgrades

**Trade-offs**:
- ✅ Safe upgrades (Docusaurus updates don't break theme)
- ✅ Minimal code maintenance
- ✅ Preserves a11y and i18n support from core
- ❌ Less control over internal component structure
- ❌ Must work within existing component APIs

**Implementation Strategy**:
1. Replace `src/pages/index.tsx` entirely (no swizzling needed for pages)
2. Create `src/components/HomepageFeatures` for module cards
3. Keep NavbarItem, Footer, etc. as pure CSS customization (no swizzling)

**Validation**: FR-002 (module overview on landing page), SC-001 (10-second comprehension).

---

### ADR-003: Icon Asset Format (SVG vs Icon Font)

**Decision**: Use inline SVG icons for module cards, not icon fonts or external SVG files.

**Context**:
- Need 4 icons for module cards (ROS 2, Gazebo, Isaac Sim, VLA)
- Clarification session confirmed: simple outline icons (minimal line-based)
- Must work in both light and dark modes with color flexibility

**Options Considered**:
1. **Inline SVG in React components (CHOSEN)**: SVG code directly in JSX
2. **External SVG files**: Import from static/img/icons/
3. **Icon font**: Custom font with icon glyphs
4. **SVG sprite sheet**: Combined SVG with <use> references

**Rationale**:
- Inline SVG allows dynamic styling via CSS (color changes for dark mode)
- No HTTP requests (faster load, no FOUC)
- Full control over accessibility (aria-label, role attributes)
- Easy to maintain (4 icons only, no sprite complexity)
- Small footprint (simple outline icons = minimal KB)

**Trade-offs**:
- ✅ Best performance (no extra requests)
- ✅ Dynamic styling (currentColor for theme integration)
- ✅ Accessibility control (proper ARIA attributes)
- ✅ Simple maintenance (4 icons, readable JSX)
- ❌ Slightly larger JSX bundle (but negligible for 4 icons)
- ❌ Can't reuse icons outside React (not a requirement)

**Implementation Example**:
```tsx
const ModuleIcon = ({ type }: { type: string }) => {
  if (type === 'ros2') {
    return (
      <svg width="48" height="48" viewBox="0 0 48 48" role="img" aria-label="ROS 2 Icon">
        <path d="M..." stroke="currentColor" fill="none" strokeWidth="2"/>
      </svg>
    );
  }
  // ... other icons
};
```

**Validation**: FR-002 (simple outline SVG icons), SC-007 (max 6 colors).

---

### ADR-004: Typography Stack Selection

**Decision**: Use system font stack with fallback to sans-serif, optimized for readability.

**Context**:
- FR-007 requires "clean, readable fonts with 16-18px body text"
- Target audience: technical professionals (AI/ML practitioners)
- Must load fast (no web font network requests)
- Needs monospace for code blocks

**Typography Choices**:

**Body Text (16px)**:
```css
font-family:
  -apple-system,           /* macOS/iOS San Francisco */
  BlinkMacSystemFont,      /* macOS/iOS fallback */
  "Segoe UI",              /* Windows */
  Roboto,                  /* Android */
  "Helvetica Neue",        /* macOS older */
  Arial,                   /* Universal fallback */
  sans-serif;              /* Generic fallback */
```

**Headings (weight: 700)**:
```css
/* Same as body but bold weight */
font-weight: 700;
letter-spacing: -0.02em;  /* Tighter for headings */
```

**Code Blocks (14px)**:
```css
font-family:
  "SF Mono",               /* macOS */
  "Consolas",              /* Windows */
  "Liberation Mono",       /* Linux */
  "Menlo",                 /* macOS older */
  monospace;               /* Generic fallback */
```

**Rationale**:
- System fonts load instantly (0ms)
- Native look-and-feel per OS (better UX)
- Excellent readability (designed for screen reading)
- Zero network cost (no web font CDN)
- Accessible across all platforms

**Trade-offs**:
- ✅ Instant load (no FOIT/FOUT issues)
- ✅ Familiar to users (system-native)
- ✅ Accessible (designed for readability)
- ✅ Zero maintenance (no font file updates)
- ❌ Not brand-distinctive (common look)
- ❌ Minor variation across OSes

**Validation**: FR-007 (clean, readable fonts), SC-003 (4.5:1 contrast), constitution readability (FK 10-12).

---

### ADR-005: Responsive Breakpoints Strategy

**Decision**: Use Docusaurus default breakpoints with mobile-first CSS approach.

**Context**:
- FR-014 requires mobile (375px+), tablet (768px+), desktop (1024px+) support
- SC-004 requires no horizontal scroll on 375px screens
- Docusaurus provides default breakpoints via Infima

**Breakpoints**:
```css
/* Docusaurus/Infima defaults */
--ifm-breakpoint-sm: 576px;   /* Small devices */
--ifm-breakpoint-md: 768px;   /* Tablets */
--ifm-breakpoint-lg: 996px;   /* Desktop */
--ifm-breakpoint-xl: 1280px;  /* Large desktop */
```

**Mobile-First Approach**:
```css
/* Base styles for mobile (375px+) */
.hero__title {
  font-size: 2rem;      /* 32px */
  padding: 1rem;
}

/* Tablet and up */
@media (min-width: 768px) {
  .hero__title {
    font-size: 3rem;    /* 48px */
    padding: 2rem;
  }
}

/* Desktop and up */
@media (min-width: 996px) {
  .hero__title {
    font-size: 4rem;    /* 64px */
    padding: 3rem;
  }
}
```

**Rationale**:
- Mobile-first = better performance (base styles smaller)
- Docusaurus breakpoints already tested and optimized
- Consistent with existing theme behavior
- Easier to maintain (no custom breakpoint system)

**Trade-offs**:
- ✅ Consistent with Docusaurus ecosystem
- ✅ Mobile-optimized (base styles lightweight)
- ✅ Well-tested breakpoints
- ❌ Not perfectly aligned with FR-014 (375px vs 576px), but 375px base covers it

**Validation**: SC-004 (no horizontal scroll at 375px), FR-014 (responsive design).

---

## Testing Strategy

### Visual Design Validation

**1. Color Contrast Testing**
- **Tool**: WebAIM Contrast Checker
- **Metric**: Minimum 4.5:1 for normal text, 3:1 for large text
- **Process**:
  1. Test all text/background combinations in light mode
  2. Test all text/background combinations in dark mode
  3. Test interactive element states (hover, focus, active)
  4. Document any failures and adjust colors

**Acceptance**: SC-003 (4.5:1 contrast ratio in both modes)

---

**2. Responsive Design Testing**
- **Tool**: Chrome DevTools Device Toolbar
- **Viewports**: 375px, 768px, 1024px, 1920px
- **Process**:
  1. Test landing page on all viewports
  2. Test chapter page on all viewports
  3. Verify no horizontal scroll at 375px
  4. Verify navigation collapsible on mobile
  5. Verify all interactive elements are touch-friendly (min 44x44px)

**Acceptance**: SC-004 (no horizontal scroll at 375px), FR-014 (responsive).

---

**3. Dark Mode Toggle Testing**
- **Process**:
  1. Toggle between light/dark modes on landing page
  2. Verify no flash of unstyled content (FOUC)
  3. Verify toggle persists across page navigation
  4. Test with system preference changes
  5. Verify all colors harmonious in both modes

**Acceptance**: SC-005 (no FOUC on toggle), FR-009 (system preference default).

---

**4. Typography Readability**
- **Tool**: Hemingway Editor, textstat (Flesch-Kincaid)
- **Metrics**: Line height 1.6+, 60-80 characters per line
- **Process**:
  1. Verify body text 16-18px across viewports
  2. Verify heading hierarchy clear (visual size differences)
  3. Measure line length at various viewports
  4. Test with browser zoom at 200%

**Acceptance**: FR-007 (16-18px body), FR-017 (heading hierarchy).

---

**5. Accessibility Testing**
- **Tool**: pa11y, axe DevTools
- **Standard**: WCAG 2.1 AA
- **Process**:
  ```bash
  pa11y http://localhost:3000 --standard WCAG2AA
  pa11y http://localhost:3000/intro --standard WCAG2AA
  # Repeat for key pages
  ```
  1. Run automated accessibility scan
  2. Test keyboard navigation (Tab, Enter, Esc)
  3. Test screen reader announcements (NVDA/JAWS)
  4. Verify focus indicators visible

**Acceptance**: Constitution requirement (WCAG 2.1 AA), SC-003 (contrast).

---

**6. Build Performance Testing**
- **Process**:
  ```bash
  time npm run build
  # Should complete in < 120 seconds
  ```
  1. Measure clean build time
  2. Measure incremental build time (single file change)
  3. Verify no build warnings or errors
  4. Check output bundle size

**Acceptance**: Constitution (< 2 min build), FR-010 (consistency).

---

**7. Landing Page Comprehension Test**
- **Manual Test**: 10-second rule
- **Process**:
  1. Show landing page to 3-5 test users (AI practitioners)
  2. Allow 10 seconds to scan page
  3. Ask: "What is this book about?", "Who is it for?", "What will you learn?"
  4. Record correct answer rate

**Acceptance**: SC-001 (10-second comprehension), SC-002 (modules in first scroll).

---

### Testing Summary Table

| Test Type | Tool | Metric | Acceptance Criteria | Frequency |
|-----------|------|--------|---------------------|-----------|
| Contrast | WebAIM | Ratio | ≥4.5:1 normal, ≥3:1 large | Pre-commit |
| Responsive | Chrome DevTools | Visual | No horiz scroll at 375px | Pre-commit |
| Dark Mode | Manual | Visual | No FOUC, smooth transition | Pre-commit |
| Typography | Visual | Sizing | 16-18px body, clear hierarchy | Pre-PR |
| Accessibility | pa11y | WCAG 2.1 AA | 0 violations | Pre-PR |
| Build Time | npm | Duration | < 120 seconds | Every commit |
| Comprehension | User test | % correct | ≥80% in 10 seconds | Pre-release |

---

## Implementation Phases

### Phase 0: Research & Design Tokens

**Goal**: Research design systems and define concrete color/typography tokens.

**Tasks**:
1. Research minimal UI design systems
   - Study: GitHub Docs, Stripe Docs, Tailwind Docs, Linear
   - Extract: color palettes, typography scales, spacing systems
   - Document: best practices for technical documentation

2. Define color tokens (neutral gray + blue accent)
   - Light mode: background, text, border, accent colors
   - Dark mode: corresponding values with proper contrast
   - Validate: all combinations pass 4.5:1 contrast ratio

3. Define typography tokens
   - Font sizes: 14px (code), 16px (body), 18-48px (headings)
   - Line heights: 1.6 (body), 1.2 (headings)
   - Weights: 400 (regular), 700 (bold)

4. Define spacing scale
   - 4px base unit (8px, 12px, 16px, 24px, 32px, 48px, 64px)

**Deliverable**: `design-tokens.md` with complete token definitions.

**Estimated Time**: Research (2-3 hours), token definition (1 hour).

---

### Phase 1: Component Specifications & Swizzling

**Goal**: Design UI components and prepare Docusaurus customization.

**Prerequisites**: Phase 0 complete (design tokens defined).

**Tasks**:
1. Design Hero section component
   - Layout: centered text, prominent CTA button
   - Content: book title, tagline, description (50-100 words)
   - Styling: gradient background or solid with subtle texture

2. Design Module Card component
   - Layout: icon (48x48), title, description (2-3 sentences)
   - Grid: 2x2 on mobile, 4x1 on desktop
   - Interaction: subtle hover effect (lift + shadow)

3. Create SVG icons for 4 modules
   - Module 1 (ROS 2): Network/nodes icon
   - Module 2 (Gazebo): Cube/3D icon
   - Module 3 (Isaac Sim): Camera/eye icon
   - Module 4 (VLA): Brain/chat bubble icon
   - Style: 2px stroke, outline only, 48x48 viewport

4. Swizzle Docusaurus components
   ```bash
   npm run swizzle @docusaurus/theme-classic HomepageFeatures -- --wrap
   ```

**Deliverable**: `component-specs.md`, 4 SVG icon files.

**Estimated Time**: Design (3-4 hours), SVG creation (2 hours).

---

### Phase 2: CSS Implementation

**Goal**: Implement custom theme via CSS variables and custom stylesheet.

**Prerequisites**: Phase 1 complete (components designed).

**Tasks**:
1. Update `src/css/custom.css` with design tokens
   ```css
   :root {
     /* Colors - Light Mode */
     --ifm-color-primary: #2563eb;
     --ifm-color-primary-dark: #1d4ed8;
     --ifm-background-color: #ffffff;
     --ifm-font-color-base: #1f2937;

     /* Typography */
     --ifm-font-size-base: 16px;
     --ifm-line-height-base: 1.6;

     /* Spacing */
     --ifm-spacing-horizontal: 1rem;
     --ifm-spacing-vertical: 1rem;
   }

   [data-theme='dark'] {
     /* Colors - Dark Mode */
     --ifm-color-primary: #60a5fa;
     --ifm-background-color: #111827;
     --ifm-font-color-base: #f9fafb;
   }
   ```

2. Add custom styles for Hero section
3. Add custom styles for Module Cards
4. Add responsive media queries
5. Validate contrast ratios for all color combinations

**Deliverable**: Updated `src/css/custom.css` (~400 lines).

**Estimated Time**: 4-6 hours.

---

### Phase 3: React Component Implementation

**Goal**: Build custom React components for landing page.

**Prerequisites**: Phase 2 complete (CSS ready).

**Tasks**:
1. Create `src/components/Hero/index.tsx`
   ```tsx
   export default function Hero() {
     return (
       <section className={styles.hero}>
         <div className="container">
           <h1 className={styles.heroTitle}>
             Physical AI & Humanoid Robotics
           </h1>
           <p className={styles.heroSubtitle}>
             A Practical Guide to ROS 2, Simulation, and Vision-Language-Action Systems
           </p>
           <div className={styles.buttons}>
             <Link className="button button--primary button--lg" to="/intro">
               Get Started →
             </Link>
           </div>
         </div>
       </section>
     );
   }
   ```

2. Create `src/components/HomepageFeatures/index.tsx`
   - Map 4 modules to cards
   - Include inline SVG icons
   - Add aria-labels for accessibility

3. Update `src/pages/index.tsx` to compose Hero + Features
   ```tsx
   export default function Home() {
     return (
       <Layout>
         <Hero />
         <main>
           <HomepageFeatures />
         </main>
       </Layout>
     );
   }
   ```

**Deliverable**: 3 new React components.

**Estimated Time**: 4-5 hours.

---

### Phase 4: Testing & Validation

**Goal**: Run all quality gates and fix issues.

**Prerequisites**: Phase 3 complete (all components implemented).

**Tasks**:
1. Run build test
   ```bash
   npm run build
   # Verify completes in < 120s with 0 errors
   ```

2. Run contrast ratio tests
   - Test 8 color combinations (light + dark modes)
   - Document results in test report

3. Run responsive design tests
   - Test 4 viewports: 375px, 768px, 1024px, 1920px
   - Capture screenshots for each

4. Run accessibility scan
   ```bash
   npm run serve  # Start local server
   pa11y http://localhost:3000 --standard WCAG2AA
   ```

5. Conduct 10-second comprehension test
   - Show to 3-5 users
   - Record feedback

6. Fix any issues found in testing

**Deliverable**: Test report with pass/fail results.

**Estimated Time**: 3-4 hours testing + 2-3 hours fixes.

---

### Phase 5: Deployment & Documentation

**Goal**: Deploy to GitHub Pages and document changes.

**Prerequisites**: Phase 4 complete (all tests passing).

**Tasks**:
1. Update `docusaurus.config.ts` if needed
   - Verify baseUrl, url settings correct
   - Verify colorMode.respectPrefersColorScheme: true

2. Commit all changes to branch
   ```bash
   git add src/ static/ docusaurus.config.ts
   git commit -m "feat: implement minimal sophisticated UI theme"
   ```

3. Push to remote and create pull request

4. Deploy preview to GitHub Pages

5. Update `docs/intro.md` status banner (if present)

6. Create before/after screenshots for PR description

**Deliverable**: GitHub PR with deployed preview link.

**Estimated Time**: 1-2 hours.

---

## Implementation Timeline

**Total Estimated Time**: 20-28 hours

| Phase | Tasks | Estimated Time |
|-------|-------|----------------|
| Phase 0: Research & Tokens | Design system research, token definition | 3-4 hours |
| Phase 1: Component Specs | Component design, SVG icons, swizzling | 5-6 hours |
| Phase 2: CSS Implementation | Custom CSS, design tokens, responsive | 4-6 hours |
| Phase 3: React Components | Hero, Module Cards, Landing Page | 4-5 hours |
| Phase 4: Testing & Validation | All quality gates, fixes | 5-7 hours |
| Phase 5: Deployment | Git, PR, GitHub Pages | 1-2 hours |

**Recommended Approach**: Execute phases sequentially. Can parallelize Phase 1 (SVG creation) and Phase 2 (CSS skeleton) if multiple contributors available.

---

## Success Criteria Validation

| Success Criterion | Validation Method | Expected Result |
|-------------------|-------------------|-----------------|
| **SC-001**: 10-second comprehension | User testing with 3-5 participants | ≥80% can answer 3 questions correctly |
| **SC-002**: Modules in first scroll | Visual test at 1920x1080 viewport | All 4 module cards visible without scrolling |
| **SC-003**: 4.5:1 contrast ratio | WebAIM Contrast Checker on 8 combinations | All pass 4.5:1 for normal text |
| **SC-004**: No horizontal scroll at 375px | Chrome DevTools mobile viewport | No overflow-x on any page |
| **SC-005**: Dark mode toggle, no FOUC | Manual toggle test | Smooth transition, no white flash |
| **SC-006**: 2-click navigation | Manual navigation test | Any chapter to any chapter ≤2 clicks |
| **SC-007**: Max 6 syntax colors | Visual inspection of code blocks | ≤6 distinct colors in theme |
| **SC-008**: Above fold at 1920x1080 | Visual test at desktop viewport | Hero + 4 modules visible without scroll |

---

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation Strategy |
|------|------------|--------|---------------------|
| **Contrast ratio failures** | Medium | High | Use WCAG-compliant color palette generator; validate early in Phase 0 |
| **Build time regression** | Low | High | Profile build before/after; avoid heavy CSS processing (stick to CSS variables) |
| **Docusaurus upgrade breaks theme** | Low | Medium | Use wrap swizzling only (safe mode); document all customizations |
| **Poor mobile experience** | Medium | Medium | Mobile-first CSS; test on real devices; use DevTools extensively |
| **SVG icons inconsistent** | Low | Low | Use single design tool (Figma/Excalidraw); maintain 2px stroke, 48x48 viewport |
| **Dark mode color clashing** | Medium | Medium | Define dark mode palette in Phase 0; test all combinations early |

---

## Next Steps

1. **Immediate**: Execute Phase 0 (Research & Design Tokens)
   - Research minimal UI design systems
   - Define color, typography, spacing tokens
   - Create `design-tokens.md`

2. **After Phase 0**: Execute Phase 1 (Component Specifications)
   - Design Hero and Module Card components
   - Create 4 SVG icons
   - Swizzle HomepageFeatures component

3. **Ongoing**: Validate against constitution requirements
   - Build time < 2 minutes
   - WCAG 2.1 AA compliance
   - No broken links

4. **Completion Target**: 3-5 days from start (assuming 5-8 hours/day)

**Recommended Next Command**: Begin implementation (no further planning commands needed).

---

**Plan Status**: READY FOR IMPLEMENTATION
**Constitution Compliance**: ✅ PASS (all requirements aligned)
**Risk Level**: LOW (well-understood technology, minimal scope)
**Blockers**: None
