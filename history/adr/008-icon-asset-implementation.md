# ADR-008: Icon Asset Implementation

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Icon System" not separate ADRs for format, loading, styling).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 002-minimal-ui-theme
- **Context:** Landing page requires 4 module card icons (ROS 2, Gazebo, Isaac Sim, VLA) that must work in both light and dark modes, load instantly without FOUC, and maintain minimal aesthetic with outline style. Icons need dynamic color styling and full accessibility support.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - defines icon system for entire site, affects loading performance
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - inline SVG, external SVG, icon fonts, sprite sheets
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - establishes pattern for all future icons and visual assets
-->

## Decision

Use **inline SVG icons** within React components, not external SVG files, icon fonts, or sprite sheets.

**Icon Implementation Stack:**
- **Format**: Inline SVG code directly in JSX components
- **Styling**: `currentColor` for dynamic theme-aware coloring
- **Accessibility**: Explicit `role="img"` and `aria-label` attributes
- **Count**: 4 icons (48x48px viewport, 2px stroke, outline style)
- **Location**: Within `src/components/HomepageFeatures/` module card component

## Consequences

### Positive

- **Best performance**: No HTTP requests (zero network latency)
- **No FOUC**: Icons render immediately with component (no loading flash)
- **Dynamic styling**: `currentColor` automatically adapts to light/dark theme
- **Full accessibility control**: Can add proper ARIA labels, roles, and descriptions
- **Simple maintenance**: Only 4 icons, readable JSX, easy to update
- **Type safety**: Icons are TypeScript-checkable as part of component code
- **Version controlled**: Icons in Git alongside component logic

### Negative

- **Slightly larger bundle**: SVG markup increases JSX file size (~1-2KB per icon)
- **Can't reuse outside React**: Icons not available as standalone files for other uses
- **No lazy loading**: Icons always bundled with component (but small enough to not matter)
- **Harder to iterate**: Requires code changes vs swapping external file

## Alternatives Considered

**Alternative 1: External SVG files**
- Pros: Reusable, cacheable, easier to swap
- Rejected: Adds HTTP requests (slower), risk of FOUC, harder to dynamically style for dark mode, can't easily add ARIA attributes

**Alternative 2: Icon font (custom or library like Font Awesome)**
- Pros: Single file, infinite scaling, widely understood
- Rejected: Extra font file download, accessibility challenges (screen readers read as text), harder to control exact styling, overkill for 4 icons

**Alternative 3: SVG sprite sheet**
- Pros: Single file for all icons, cacheable, reusable with `<use>`
- Rejected: Still requires HTTP request, more complex implementation, unnecessary for only 4 icons, FOUC risk

**Why Inline SVG Chosen:**
- Zero network overhead (most critical for landing page performance)
- Guaranteed no FOUC (icons render with initial HTML)
- `currentColor` makes dark mode trivial (inherits from CSS theme)
- Full ARIA control ensures accessibility compliance (WCAG 2.1 AA)
- Only 4 icons = minimal bundle impact (~4-8KB total)
- Simpler implementation than sprite or font systems

## References

- Feature Spec: [specs/002-minimal-ui-theme/spec.md](../../specs/002-minimal-ui-theme/spec.md) (FR-002, Clarifications)
- Implementation Plan: [specs/002-minimal-ui-theme/plan.md](../../specs/002-minimal-ui-theme/plan.md) (ADR-003, lines 213-258)
- Related ADRs: ADR-006 (Theming), ADR-007 (Components)
- Success Criteria: SC-007 (max 6 colors), SC-008 (above fold load), FR-002 (simple outline SVG icons)
