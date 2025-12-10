# ADR-014: Frontend Integration Approach for Docusaurus

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-10
- **Feature:** rag-chatbot
- **Context:** Chat widget must integrate into existing Docusaurus 3.9.2 book site without breaking builds, survive Docusaurus version upgrades, and support responsive design across desktop/tablet/mobile. Existing project uses React 19, TypeScript 5.6.2, and Docusaurus classic theme.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - determines upgrade path, maintenance burden, Docusaurus version coupling
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - swizzling vs custom plugin vs theme modification
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - affects all frontend work (Phase 3), builds, theming, deployment
-->

## Decision

**Frontend Integration Stack:**
- **Integration Method**: Docusaurus theme swizzling (Root component ejection)
- **Component Structure**:
  - **Main Component**: `src/components/ChatWidget/ChatWidget.tsx`
  - **Injection Point**: `src/theme/Root.tsx` (swizzled from `@docusaurus/theme-classic`)
  - **Hooks**: `useChat.ts`, `useTextSelection.ts` (custom React hooks)
  - **API Client**: `src/api/chatClient.ts` (TypeScript SSE client)
- **Styling Approach**: CSS Modules (`ChatWidget.module.css`) with Docusaurus CSS variables
- **Theme Compatibility**: Use `var(--ifm-color-*)` variables for dark mode support
- **Global Injection**: `<ChatWidget />` rendered once in Root, available on all pages

**Swizzling Command:**
```bash
npm run swizzle @docusaurus/theme-classic Root -- --eject
```

**Root Component:**
```typescript
// src/theme/Root.tsx
import React from 'react';
import ChatWidget from '../components/ChatWidget/ChatWidget';

export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatWidget />  // Injected globally
    </>
  );
}
```

## Consequences

### Positive

- **Upgrade-Safe**: Swizzling is official Docusaurus pattern; survives version upgrades better than hacks
- **Non-Invasive**: No modifications to Docusaurus core files or theme internals
- **Global Availability**: Widget accessible on every page without per-page imports
- **Theme Integration**: CSS variables (`--ifm-color-primary`, `--ifm-background-color`) ensure consistency with site theme
- **Dark Mode Support**: Automatically inherits Docusaurus dark/light mode via CSS vars
- **Build Safety**: Swizzled component part of normal build process; errors caught at build time
- **Easy Removal**: Delete `src/theme/Root.tsx` to remove widget without touching Docusaurus core

### Negative

- **Swizzle Maintenance**: If Docusaurus updates Root component, must manually merge changes into swizzled version
- **Full Page Bundle**: ChatWidget code loaded on every page even if user never opens it (adds ~20KB to bundle)
- **Limited Isolation**: Widget shares same React context as Docusaurus (could conflict if both use same context providers)
- **Ejection Coupling**: Ejected Root component couples to Docusaurus theme internals (wrapping vs unsafe modes)
- **No Lazy Loading**: Widget always included in initial page load (can't code-split by route)

## Alternatives Considered

**Alternative 1: Custom Docusaurus Plugin**
- Create `plugins/rag-chatbot-plugin/` with custom plugin lifecycle
- Pros: More control, plugin-level configuration, conditional loading
- Cons: Complex setup, fragile across Docusaurus versions, requires understanding plugin API, harder to debug
- Rejected: Over-engineering for simple global component; swizzling is simpler and more maintainable

**Alternative 2: Direct Theme Modification (fork classic theme)**
- Fork `@docusaurus/theme-classic`, modify directly
- Pros: Maximum control, no swizzling
- Cons: Lose upstream updates, hard to merge Docusaurus upgrades, breaks `npm update`
- Rejected: Maintenance nightmare; defeats purpose of using Docusaurus

**Alternative 3: Swizzle Specific Layout Components (DocPage, Layout)**
- Swizzle `DocPage` or `Layout` instead of Root
- Pros: More targeted injection, less coupling to Root changes
- Cons: Must inject in multiple layouts (DocPage, BlogPage, etc.), miss some pages, inconsistent availability
- Rejected: Root injection simpler (single point of injection), widget should be global anyway

**Alternative 4: Script Tag Injection (document.body.appendChild)**
- Add `<script>` tag in `docusaurus.config.ts` that injects widget via DOM manipulation
- Pros: Zero Docusaurus coupling, works with any theme
- Cons: Not React-idiomatic, hydration issues, harder to debug, can't use React hooks, breaks SSR
- Rejected: Fights React paradigm; loses type safety and React dev tools

**Alternative 5: Markdown/MDX Component (include in every page)**
- Create MDX component, import in every page frontmatter
- Pros: Per-page control
- Cons: Tedious (must add to all 21 pages), easy to forget on new pages, not globally available
- Rejected: Violates DRY principle; widget should be global (not page-specific)

**Why Docusaurus Root Swizzling Chosen:**
- Official Docusaurus pattern (documented, supported)
- Simplest way to achieve global component injection
- Non-invasive (doesn't modify Docusaurus core files)
- Type-safe (TypeScript React components)
- Theme-integrated (CSS variables ensure visual consistency)
- Easy to maintain (swizzled file is small, clear purpose)

## References

- Feature Spec: [specs/rag-chatbot/spec.md](../../specs/rag-chatbot/spec.md) (FR-001: Chat widget UI component, FR-027: Must not break Docusaurus build)
- Implementation Plan: [specs/rag-chatbot/plan.md](../../specs/rag-chatbot/plan.md) (Lines 46-51: Frontend Integration Strategy, Task 3.7: Docusaurus Integration)
- Related ADRs: ADR-015 (Streaming Protocol - frontend client), ADR-011 (Backend Stack - API contract)
- Docusaurus Docs: https://docusaurus.io/docs/swizzling (Official swizzling guide)
- Evaluator Evidence: plan.md lines 1406-1434 (Phase 3 checkpoint verifies integration works without breaking build)
