# ADR-009: Typography System

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Typography Stack" not separate ADRs for body font, heading font, code font).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 002-minimal-ui-theme
- **Context:** The book requires clean, readable typography for technical content targeting AI/ML practitioners. Must load instantly (<3sec page load), maintain 4.5:1 contrast ratio, and support body text (16-18px), headings, and code blocks (monospace). Constitution requires Flesch-Kincaid 10-12 readability.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - affects readability, accessibility, and performance across all pages
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - system fonts, web fonts (Google Fonts, Adobe Fonts), custom fonts
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - typography impacts every page, all content, and user reading experience
-->

## Decision

Use **system font stack** (no web fonts) with OS-native sans-serif for body/headings and monospace for code.

**Typography Stack Components:**
- **Body Text**: `-apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif` (16px)
- **Headings**: Same stack with `font-weight: 700` and `letter-spacing: -0.02em`
- **Code Blocks**: `"SF Mono", "Consolas", "Liberation Mono", "Menlo", monospace` (14px)
- **Line Height**: 1.6 for body, 1.2 for headings
- **Font Sizes**: 14px (code), 16px (body), 18-48px (headings scale)

## Consequences

### Positive

- **Instant load**: Zero network latency (0ms font load time)
- **Familiar UX**: Native OS fonts feel natural to users on each platform
- **Excellent readability**: System fonts designed by Apple/Microsoft/Google specifically for screen reading
- **Zero network cost**: No CDN requests, no font file downloads, no bandwidth usage
- **No FOIT/FOUT**: Never see "flash of invisible text" or "flash of unstyled text"
- **Zero maintenance**: No font file updates, license management, or versioning
- **Consistent across site**: Same font stack used by Docusaurus core (no conflicts)

### Negative

- **Not brand-distinctive**: Common look shared by many documentation sites
- **Minor cross-platform variation**: San Francisco (macOS) vs Segoe UI (Windows) vs Roboto (Android) have subtle differences
- **No custom personality**: Can't express unique brand identity through typography
- **Less designer control**: Can't fine-tune exact letterforms, kerning, or weights

## Alternatives Considered

**Alternative 1: Google Fonts (e.g., Inter, Roboto, Open Sans)**
- Pros: Consistent cross-platform, more design control, popular choices
- Rejected: Adds 100-300ms network latency, increases page load time, external CDN dependency, privacy concerns (Google tracking)

**Alternative 2: Adobe Fonts (Typekit)**
- Pros: High-quality typography, professional look
- Rejected: Requires subscription, adds network latency, licensing complexity, overkill for documentation

**Alternative 3: Self-hosted custom fonts**
- Pros: No external CDN, brand-distinctive
- Rejected: Still adds load time (even from same domain), requires font file management, increases bundle size, unnecessary complexity

**Why System Fonts Chosen:**
- Optimal performance (0ms = fastest possible)
- Native OS feel improves usability (users accustomed to these fonts)
- Meets constitution readability requirement (system fonts designed for reading)
- Zero dependencies = simpler maintenance
- Aligns with minimal, sophisticated theme goal (no unnecessary flourishes)
- Proven approach (used by GitHub Docs, Apple Developer Docs, Stripe Docs)

## References

- Feature Spec: [specs/002-minimal-ui-theme/spec.md](../../specs/002-minimal-ui-theme/spec.md) (FR-007, FR-017)
- Implementation Plan: [specs/002-minimal-ui-theme/plan.md](../../specs/002-minimal-ui-theme/plan.md) (ADR-004, lines 261-318)
- Constitution: [.specify/memory/constitution.md](../../.specify/memory/constitution.md) (Accessibility Section V, Flesch-Kincaid 10-12)
- Related ADRs: ADR-006 (Theming), ADR-010 (Responsive Design)
- Success Criteria: SC-003 (4.5:1 contrast), FR-007 (16-18px body), Page load <3sec
