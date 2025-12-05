# AI/Spec-Driven Book Creation Constitution

## Core Principles

### I. Accuracy and Source Transparency
- All technical claims, concepts, and explanations must include links to official documentation or authoritative sources
- No unsupported assertions or opinions presented as facts
- When multiple authoritative sources conflict, cite both and explain the difference
- Regular fact-checking required for all technical content
- Citations must use stable URLs where possible (not subject to frequent changes)

### II. Consistency Across Chapters
- Every chapter must follow the standardized layout: Overview → Concepts → Examples → References
- Voice, tone, and terminology must remain uniform throughout all modules
- Code style and formatting conventions must be consistent across all examples
- Navigation patterns and cross-references must follow the same structure
- All assets (images, diagrams, code samples) must adhere to naming and organizational conventions

### III. Modular Structure (NON-NEGOTIABLE)
- Content must align with Docusaurus conventions and best practices
- Each module contains exactly 4 chapters
- Total book structure: 4 modules × 4 chapters = 16 chapters
- File structure must follow Docusaurus sidebar and routing rules strictly
- Each module and chapter must be independently readable while maintaining narrative flow
- Sidebar configuration must be maintained in `sidebars.js`

### IV. Code Quality and Testing
- All code samples must be tested before inclusion in the book
- Code examples must be runnable and produce expected outputs
- Include setup instructions for any dependencies or environment requirements
- Prefer simple, self-contained examples over complex, multi-file setups
- Code blocks must specify language for proper syntax highlighting
- Error cases and edge conditions should be demonstrated where relevant

### V. Accessibility and Clarity
- **Readability**: Flesch-Kincaid grade level 10-12 (check with readability tools)
- **Sentence structure**: Max 25 words/sentence average per section
- **Active voice**: 75%+ of sentences use active voice
- Target audience: beginners and intermediate learners
- **Technical jargon**: Define within 1 paragraph of first use
- **Complex concepts**: Break into 2+ progressive examples with difficulty labels
- **Visual aids**: Min 1 diagram/table per 500 words for abstract concepts
- Each chapter should build on previous knowledge without assuming expertise

### VI. Deployment and Build Standards
- The book must build cleanly with `npm run build` with zero errors
- No broken links (internal or external)
- All images and assets must load correctly
- Build process must complete in reasonable time (< 2 minutes for full build)
- GitHub Pages deployment must succeed without manual intervention
- All assets must be compatible with GitHub Pages hosting constraints

### VII. Originality and Attribution
- **Plagiarism check**: All content must pass Turnitin/Copyscape (< 15% similarity excluding quotes)
- **Code attribution**: Any code adapted from external sources must include:
  - Inline comment with source URL
  - License compatibility verification
  - Modification description if altered
- **Quote policy**: Direct quotes > 2 sentences require block quote formatting + citation
- **AI-generated content**: Must be reviewed, tested, and verified by human author
- **Self-plagiarism**: Reusing own content from other projects requires disclosure

## Content Standards

### Chapter Structure Requirements
Each chapter must include:
1. **Overview Section**: Brief introduction, learning objectives, prerequisites
2. **Concepts Section**: Core explanations with supporting examples
3. **Examples Section**: Practical, hands-on demonstrations
4. **References Section**: Links to official docs, further reading, related chapters

### Code Sample Standards
- Maximum 30 lines per code block (split longer examples)
- Include comments for non-obvious logic
- Show both correct usage and common mistakes where helpful
- Provide context: what problem does this solve?
- Test output included or described

### Asset Management
- Images: PNG or SVG format, max 500KB per file
- Diagrams: Created with consistent tooling (Excalidraw, Mermaid, or similar)
- All assets stored in `static/img/` organized by module
- Naming convention: `module-XX-chapter-YY-description.ext`

## Technical Constraints

### Build Environment
- Node.js version: 18.x or higher
- Docusaurus version: 3.x
- Package manager: npm
- Target deployment: GitHub Pages

### File Organization
```
docs/
├── module-01/
│   ├── chapter-01.md
│   ├── chapter-02.md
│   ├── chapter-03.md
│   └── chapter-04.md
├── module-02/
├── module-03/
└── module-04/
static/
├── img/
└── assets/
```

### Performance Requirements
- Page load time: < 3 seconds on standard connection
- Image optimization: all images compressed and optimized
- Minimal external dependencies that could slow builds
- Search functionality must be responsive

## Development Workflow

### Content Creation Process
1. Draft chapter in markdown following template
2. Add all code examples and test them locally
3. Include citations and verify all links
4. Add images and optimize them
5. Build locally and verify no errors
6. Review against checklist before commit

### Quality Gates
Before any commit:
- [ ] All code samples tested and working
- [ ] All links verified (no 404s)
- [ ] Builds successfully with `npm run build`
- [ ] No console warnings or errors
- [ ] Chapter follows standard structure
- [ ] All technical claims cited
- [ ] Images optimized and loading correctly
- [ ] Cross-references are accurate

### Review Requirements
- **Technical accuracy review**:
  - 2+ reviewers test all code samples in clean environment
  - All claims verified against cited sources
  - Pass criteria: Zero factual errors, all code runs successfully
- **Style consistency audit**:
  - Run against style guide checklist (all checkboxes must pass)
  - Terminology matches glossary (auto-check with linter)
- **Automated checks** (must pass):
  - `npm run build` → zero errors
  - Link checker → zero 404s (internal), < 5% broken (external with documented exceptions)
  - Readability score → Flesch-Kincaid 10-12
  - Image size check → all < 500KB
- **Accessibility audit**: WCAG 2.1 AA compliance for navigation

## Governance

### Constitution Authority
- This constitution supersedes all other development practices for this project
- All content creation, code samples, and structural decisions must comply
- Any deviations require documented justification and approval
- Amendments to this constitution require version increment and changelog entry

### Compliance Verification
- Every PR must include checklist confirming adherence to principles
- Automated checks enforce build success and link validity
- Manual review verifies content quality and consistency
- Failed quality gates block merge until resolved

### Change Management
- Breaking changes to structure require migration plan
- Template updates must be applied retroactively to existing chapters
- Version bumps follow semantic versioning for constitution itself

**Version**: 1.0.0 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
