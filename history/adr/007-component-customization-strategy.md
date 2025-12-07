# ADR-007: Component Customization Strategy

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Component Strategy" not separate ADRs for swizzling, pages, layouts).

- **Status:** Accepted
- **Date:** 2025-12-07
- **Feature:** 002-minimal-ui-theme
- **Context:** The landing page requires custom hero section and module cards beyond Docusaurus defaults. Docusaurus provides "swizzling" to customize theme components with two modes: "eject" (full control, breaks on updates) vs "wrap" (safe, preserves compatibility). Must balance customization needs with long-term maintainability.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - affects Docusaurus upgrade path and maintenance burden
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - wrap swizzling, eject swizzling, CSS-only, custom plugin
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - determines component architecture for all custom UI elements
-->

## Decision

Use **"wrap" (safe) swizzling** for theme components and **full page replacement** for landing page, avoiding "eject" (unsafe) swizzling.

**Component Customization Stack:**
- **Landing Page**: Full replacement via `src/pages/index.tsx` (no swizzling needed)
- **Module Cards**: Wrap-swizzled `HomepageFeatures` component
- **Hero Section**: New custom component in `src/components/Hero/`
- **Navigation/Footer**: CSS-only customization (no swizzling)
- **Styling**: CSS Modules for component-specific styles

## Consequences

### Positive

- **Safe Docusaurus upgrades**: Wrap swizzling preserves compatibility with future versions
- **Minimal maintenance**: Core theme components auto-update, only custom wrapping logic needs maintenance
- **Preserves accessibility**: Inherits a11y features from Docusaurus core components
- **Preserves i18n support**: Maintains internationalization capabilities from core
- **Clear separation**: Easy to identify custom code vs framework code
- **Reduced risk**: Won't break on Docusaurus patch/minor version updates

### Negative

- **Less control**: Can't modify internal component structure, must work within existing APIs
- **Wrapper complexity**: Some customizations require understanding component composition
- **API constraints**: Must work within Docusaurus component prop interfaces
- **Limited deep customization**: Can't fundamentally alter core component behavior

## Alternatives Considered

**Alternative 1: Eject (unsafe) swizzling**
- Pros: Full control over component source, can modify anything
- Rejected: Breaks on Docusaurus updates, high maintenance burden, loses automatic bug fixes and accessibility improvements

**Alternative 2: CSS-only (no swizzling)**
- Pros: Simplest, zero component code changes
- Rejected: Insufficient for landing page module cards and hero section (requires structural HTML changes)

**Alternative 3: Custom Docusaurus plugin**
- Pros: Most flexible, complete control
- Rejected: Overkill for theme customization, adds complexity, harder for contributors to understand

**Why Wrap Swizzling + Page Replacement Chosen:**
- Balances customization needs (landing page hero + module cards) with maintainability
- `src/pages/index.tsx` can be fully replaced without swizzling (Docusaurus allows page overrides)
- Wrap swizzling provides safe customization for `HomepageFeatures` without breaking updates
- CSS-only approach sufficient for nav/footer (no component changes needed)
- Minimizes code that could break on Docusaurus upgrades

## References

- Feature Spec: [specs/002-minimal-ui-theme/spec.md](../../specs/002-minimal-ui-theme/spec.md) (FR-001 to FR-005, Key Entities)
- Implementation Plan: [specs/002-minimal-ui-theme/plan.md](../../specs/002-minimal-ui-theme/plan.md) (ADR-002, lines 176-210)
- Docusaurus Swizzling Docs: https://docusaurus.io/docs/swizzling
- Related ADRs: ADR-006 (Theming Approach), ADR-008 (Icon Assets)
- Success Criteria: FR-002 (module overview), SC-001 (10-second comprehension), SC-006 (2-click navigation)
