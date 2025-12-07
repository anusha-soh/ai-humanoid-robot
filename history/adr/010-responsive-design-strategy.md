# ADR-010: Responsive Design Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Responsive System" not separate ADRs for breakpoints, mobile-first, grid).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 002-minimal-ui-theme
- **Context:** The book must work on mobile (375px+), tablet (768px+), and desktop (1024px+) devices. Constitution requires accessibility and usability across all viewports. Must avoid horizontal scrolling on narrow screens while maintaining readable typography and touch-friendly interactions.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines responsive architecture for all pages and components
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - mobile-first vs desktop-first, custom breakpoints vs framework defaults, CSS Grid vs Flexbox
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects layout of every page and component, impacts CSS organization
-->

## Decision

Use **Docusaurus default breakpoints** with **mobile-first CSS** approach, leveraging Infima's existing responsive system.

**Responsive Design Stack:**
- **Breakpoints**: Infima defaults (576px, 768px, 996px, 1280px)
- **Approach**: Mobile-first (base styles for 375px+, progressive enhancement via `min-width` media queries)
- **Layout**: CSS Flexbox for components, CSS Grid for page-level layouts
- **Base Viewport**: 375px minimum (iPhone SE, small Android phones)
- **Typography Scaling**: Fluid sizing (16px mobile → 18px desktop)

## Consequences

### Positive

- **Consistent with Docusaurus**: Uses same breakpoints as core theme (no conflicts)
- **Mobile-optimized**: Base styles are lightweight (faster on mobile devices)
- **Well-tested**: Infima breakpoints proven across thousands of Docusaurus sites
- **Easy maintenance**: No custom breakpoint system to manage
- **Progressive enhancement**: Mobile users get usable site even if large media queries fail
- **Future-proof**: Works with new device sizes (mobile-first scales up naturally)
- **Familiar to contributors**: Standard web development approach

### Negative

- **Not perfectly aligned with spec**: FR-014 specifies 375px minimum, but Infima's first breakpoint is 576px (still works fine with mobile-first base)
- **Less granular**: Can't fine-tune for specific device sizes (e.g., separate iPad Pro breakpoint)
- **Inherited constraints**: Tied to Infima's responsive system (can't easily change breakpoints without forking)

## Alternatives Considered

**Alternative 1: Custom breakpoints (360px, 768px, 1024px, 1440px)**
- Pros: Perfectly aligned with spec requirements, more control
- Rejected: Conflicts with Docusaurus core, harder to maintain, reinvents Infima's system, breaks ecosystem tooling

**Alternative 2: Desktop-first (max-width media queries)**
- Pros: Easier to design (start with full-featured desktop version)
- Rejected: Worse mobile performance (large base styles), harder to progressively enhance, outdated approach

**Alternative 3: Container queries (new CSS feature)**
- Pros: Component-based responsiveness, more flexible than media queries
- Rejected: Limited browser support (not in Safari 15), Docusaurus doesn't use container queries, would create inconsistency

**Alternative 4: CSS-in-JS responsive utilities (styled-components breakpoints)**
- Pros: Type-safe breakpoints, JavaScript-based
- Rejected: Already decided on CSS variables approach (ADR-006), adds runtime overhead, conflicts with theming decision

**Why Docusaurus Breakpoints + Mobile-First Chosen:**
- Leverages battle-tested Infima system (no bugs, widely used)
- Mobile-first = better performance (critical for global audience)
- 375px base viewport requirement met (base styles support it, first breakpoint at 576px just adds enhancements)
- Simpler implementation (no custom system to maintain)
- Easier for contributors (familiar approach, documented in Docusaurus)

## Implementation Notes

**Base Styles (375px - 575px)**:
- Single column layouts
- 16px body text
- Full-width navigation
- Touch-friendly buttons (44x44px minimum)

**Small (576px+)**:
- Wider containers
- 17px body text

**Medium (768px+)**:
- Two-column layouts where appropriate
- Module cards: 2×2 grid
- 18px body text
- Expanded navigation

**Large (996px+)**:
- Three-column layouts
- Module cards: 4×1 grid
- Side-by-side content + sidebar
- Full navigation visible

**Extra Large (1280px+)**:
- Max-width containers (prevent ultra-wide lines)
- Larger spacing/padding

## References

- Feature Spec: [specs/002-minimal-ui-theme/spec.md](../../specs/002-minimal-ui-theme/spec.md) (FR-014, SC-004)
- Implementation Plan: [specs/002-minimal-ui-theme/plan.md](../../specs/002-minimal-ui-theme/plan.md) (ADR-005, lines 321-377)
- Docusaurus Infima Docs: https://infima.dev/docs/layout/breakpoints
- Related ADRs: ADR-006 (Theming), ADR-007 (Components), ADR-009 (Typography)
- Success Criteria: SC-004 (no horizontal scroll at 375px), FR-014 (responsive design), SC-008 (above fold on 1920x1080)
