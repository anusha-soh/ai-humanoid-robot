# ADR-006: Docusaurus Theming Approach

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Theming System" not separate ADRs for CSS, dark mode, responsive).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 002-minimal-ui-theme
- **Context:** The Physical AI book site requires a minimal, sophisticated UI theme with neutral gray palette + blue accents, system preference-aware dark mode, and responsive design. Must maintain Docusaurus compatibility, build time <2min, and WCAG 2.1 AA accessibility.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines all styling architecture, affects every page
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - CSS variables, Styled Components, Tailwind, SASS/LESS
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - impacts all components, pages, and future styling work
-->

## Decision

Use **Docusaurus CSS Custom Properties** (CSS variables) with Infima framework for theme customization, avoiding CSS-in-JS libraries or preprocessors.

**Theming Stack Components:**
- **Styling**: CSS custom properties (Infima variables) in `src/css/custom.css`
- **Dark Mode**: Native `[data-theme='dark']` selector with system preference detection
- **Component Scoping**: CSS Modules for custom React components
- **Responsive Design**: Docusaurus default breakpoints (576px, 768px, 996px, 1280px) with mobile-first approach

## Consequences

### Positive

- **Zero runtime overhead**: Pure CSS (no JavaScript runtime for styling)
- **Fastest build time**: No CSS-in-JS processing, stays within <2min requirement
- **Native dark mode**: Automatic support via Docusaurus theming without additional libraries
- **Smallest bundle**: No extra dependencies beyond Docusaurus core
- **Easy maintenance**: Override only changed variables, inherit rest from Infima
- **Contributor-friendly**: Standard CSS familiar to all web developers
- **Upgrade-safe**: Docusaurus-native approach won't break on version updates

### Negative

- **Less type safety**: No TypeScript checking for CSS compared to styled-components
- **Manual scoping**: Must use CSS Modules or BEM for component styles (no automatic scoping)
- **Limited dynamic styling**: Can't compute styles in JavaScript as easily
- **No co-location**: Styles separate from components (unless using CSS Modules)

## Alternatives Considered

**Alternative 1: Styled Components (CSS-in-JS)**
- Pros: TypeScript support, automatic scoping, co-located styles
- Rejected: Adds runtime overhead, increases build time, larger bundle size, may conflict with Docusaurus SSR

**Alternative 2: Tailwind CSS (utility-first)**
- Pros: Rapid development, consistent spacing/colors, built-in responsiveness
- Rejected: Requires build configuration changes, large HTML bloat with utility classes, steeper learning curve for contributors

**Alternative 3: SASS/LESS (preprocessors)**
- Pros: Variables, nesting, mixins
- Rejected: CSS variables already provide variable support, adds build step, Infima already uses CSS variables

**Why CSS Custom Properties Chosen:**
- Leverages existing Docusaurus/Infima infrastructure (no new dependencies)
- Optimal performance (zero runtime, minimal build impact)
- Native dark mode support via `prefers-color-scheme` + Docusaurus theming
- Maintains <2min build time requirement
- Easy for contributors familiar with standard CSS

## References

- Feature Spec: [specs/002-minimal-ui-theme/spec.md](../../specs/002-minimal-ui-theme/spec.md) (FR-006 to FR-010)
- Implementation Plan: [specs/002-minimal-ui-theme/plan.md](../../specs/002-minimal-ui-theme/plan.md) (ADR-001, lines 124-173)
- Related ADRs: ADR-007 (Component Customization), ADR-008 (Icon Assets)
- Success Criteria: SC-003 (4.5:1 contrast), SC-005 (no FOUC on dark mode toggle), Build time <2min
