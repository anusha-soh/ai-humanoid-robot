# Implementation Plan: Physical AI & Humanoid Robotics Book

**Branch**: `001-physical-ai-book` | **Date**: 2025-12-05 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-physical-ai-book/spec.md`

## Summary

Create a comprehensive 16-chapter book (4 modules × 4 chapters) teaching Physical AI and Humanoid Robotics to beginner AI practitioners with no robotics background. The book follows a simulation-first approach, building from ROS 2 foundations through physics simulation, AI perception, to voice-controlled autonomous systems. All content delivered via Docusaurus static site deployed to GitHub Pages, with rigorous quality standards (Flesch-Kincaid 10-12, 75%+ active voice, <15% plagiarism, MCP server contacx7 verification).

**Technical Approach**: Structure-first iterative development. Build complete book structure with placeholder chapters first, then populate with real content module-by-module. This allows early deployment, navigation testing, and progressive content delivery.

## Technical Context

**Platform**: Docusaurus 3.x static site generator
**Build Environment**: Node.js 18.x+, npm package manager
**Language/Version**: Markdown (CommonMark), Python 3.10+ (for code examples), Bash (build scripts)
**Primary Dependencies**:
- Docusaurus 3.x (site generation)
- MCP server contacx7 (information verification)
- Readability tools (textstat, Hemingway Editor)
- Plagiarism checker (Turnitin/Copyscape API)
- Link checker (broken-link-checker)
- WCAG validator (pa11y)

**Content Dependencies** (what readers learn):
- ROS 2 Humble LTS
- Gazebo Classic 11
- NVIDIA Isaac Sim 2023.1+
- Python 3.10+, PyTorch 2.x
- OpenAI Whisper, LangChain/OpenAI API

**Storage**: Static markdown files in `docs/` directory, assets in `static/img/`
**Testing**:
- Readability: textstat (Flesch-Kincaid 10-12)
- Code: pytest for all examples, manual execution validation
- Build: npm run build (must complete <2min, zero errors)
- Links: broken-link-checker (zero internal 404s)
- Plagiarism: Copyscape API (<15% similarity)
- Accessibility: pa11y (WCAG 2.1 AA)

**Target Platform**: GitHub Pages static hosting (web browser, responsive design)
**Project Type**: Documentation site (Docusaurus book structure)
**Performance Goals**:
- Page load <3 seconds
- Build time <2 minutes
- Image optimization: all <500KB

**Constraints**:
- Simulation-only (zero hardware requirements)
- Beginner-friendly (Flesch-Kincaid 10-12, no prior robotics assumed)
- Constitution-compliant (16 chapters exactly, specific quality metrics)
- Content verified via MCP server contacx7 before publication

**Scale/Scope**:
- 16 chapters across 4 modules
- ~50-80 pages per module (200-320 total pages)
- 48-64 code examples (3-4 per chapter)
- 16 exercises (1 per chapter)
- 1 capstone project (Chapter 16)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Mandatory Requirements (from constitution)

- [x] **4 modules × 4 chapters = 16 chapters** (Constitution Section III)
- [x] **Chapter structure**: Overview → Concepts → Examples → References (Constitution Section III)
- [x] **Flesch-Kincaid grade 10-12** (Constitution Section V)
- [x] **75%+ active voice, max 25 words/sentence** (Constitution Section V)
- [x] **Max 30 lines per code block** (Constitution Section IV)
- [x] **All code tested and runnable** (Constitution Section IV)
- [x] **MCP server contacx7 verification** (Constitution Section III)
- [x] **<15% plagiarism similarity** (Constitution Section VII)
- [x] **Build time <2 minutes** (Constitution Section VI)
- [x] **PNG/SVG images, max 500KB** (Constitution Section VI)
- [x] **GitHub Pages deployment** (Constitution Section VI)

### Violations: None

All requirements align with constitution standards. No complexity justifications needed.

## Project Structure

### Documentation (this feature)

```text
specs/001-physical-ai-book/
├── spec.md                     # Feature specification (complete)
├── plan.md                     # This file (implementation plan)
├── QUALITY_VALIDATION.md       # Quality assessment report
├── IMPLEMENTATION_LEAK_FIXES.md # Implementation cleanup log
└── checklists/
    └── requirements.md         # Spec validation checklist
```

### Source Code (repository root)

```text
docs/                           # Docusaurus content root
├── intro.md                    # Landing page
├── module-01/                  # Module 1: Robotic Nervous System (ROS 2)
│   ├── chapter-01.md          # Ch 1: Welcome to Physical AI
│   ├── chapter-02.md          # Ch 2: ROS 2 Architecture
│   ├── chapter-03.md          # Ch 3: Describing Robots (URDF)
│   └── chapter-04.md          # Ch 4: Python to ROS (rclpy)
├── module-02/                  # Module 2: Digital Twin (Gazebo & Unity)
│   ├── chapter-05.md          # Ch 5: Physics Simulation Fundamentals
│   ├── chapter-06.md          # Ch 6: Building First Humanoid Simulation
│   ├── chapter-07.md          # Ch 7: Sensor Simulation
│   └── chapter-08.md          # Ch 8: High-Fidelity Rendering (Unity)
├── module-03/                  # Module 3: AI-Robot Brain (NVIDIA Isaac)
│   ├── chapter-09.md          # Ch 9: Intro to Isaac Sim
│   ├── chapter-10.md          # Ch 10: Perception Pipelines
│   ├── chapter-11.md          # Ch 11: Visual SLAM & Localization
│   └── chapter-12.md          # Ch 12: Path Planning (Nav2)
├── module-04/                  # Module 4: Vision-Language-Action (VLA)
│   ├── chapter-13.md          # Ch 13: Voice Commands (Whisper)
│   ├── chapter-14.md          # Ch 14: LLMs as Task Planners
│   ├── chapter-15.md          # Ch 15: Multimodal Integration
│   └── chapter-16.md          # Ch 16: Capstone - Autonomous Butler
└── appendix/
    ├── prerequisites.md        # Prerequisites refresher
    ├── glossary.md            # Technical glossary
    ├── troubleshooting.md     # Common issues and solutions
    └── resources.md           # Further reading and links

static/
├── img/                        # All images organized by module
│   ├── module-01/             # Module 1 images
│   │   ├── chapter-01/        # Chapter-specific images
│   │   ├── chapter-02/
│   │   ├── chapter-03/
│   │   └── chapter-04/
│   ├── module-02/             # Module 2 images
│   │   └── ... (same pattern)
│   ├── module-03/
│   └── module-04/
└── assets/
    └── diagrams/              # Mermaid/Excalidraw source files

docusaurus.config.js           # Site configuration
sidebars.js                    # Navigation structure
package.json                   # Dependencies
.github/
└── workflows/
    ├── build-deploy.yml       # CI/CD for GitHub Pages
    ├── quality-check.yml      # Automated quality gates
    └── link-checker.yml       # Weekly link validation

scripts/                        # Build and validation scripts
├── validate-readability.js    # Flesch-Kincaid checker
├── validate-code-examples.sh  # Code execution tester
├── check-plagiarism.js        # Plagiarism API wrapper
└── optimize-images.sh         # Image compression pipeline
```

**Structure Decision**: Docusaurus book structure with modular organization. Each module is a separate directory with 4 chapters. Assets organized by module/chapter for maintainability. Automated quality gates via GitHub Actions ensure constitution compliance on every commit.

## Complexity Tracking

> **No violations detected - this section intentionally left empty**

All requirements align with constitution. No complexity justifications needed.

## Key Architectural Decisions

### ADR-001: Simulation Tool Strategy (Gazebo + Isaac Sim Hybrid)

**Decision**: Use Gazebo Classic for Modules 1-2 (foundation, basic simulation), transition to NVIDIA Isaac Sim for Modules 3-4 (advanced perception, VLA).

**Context**:
- Gazebo: Lightweight, easier setup, good for ROS 2 basics
- Isaac Sim: GPU-accelerated, photorealistic, better for ML/AI workloads
- Spec requires "zero hardware requirements" but allows cloud alternatives for GPU

**Options Considered**:
1. **Gazebo-only**: Simpler setup, but weak perception/rendering
2. **Isaac-only**: Best quality, but heavy GPU requirements exclude some readers
3. **Hybrid (CHOSEN)**: Progressive complexity - start simple (Gazebo), advance to Isaac
4. **Multiple alternatives**: Too many tools, cognitive overload

**Rationale**:
- Gazebo in Modules 1-2 minimizes setup friction for beginners
- Isaac in Modules 3-4 provides photorealism needed for computer vision/VLA
- Readers build confidence with Gazebo before tackling Isaac requirements
- Edge case: Provide Gazebo-only exercises in Module 3-4 for readers without GPU

**Trade-offs**:
- ✅ Progressive learning curve
- ✅ Accessible to readers without high-end GPUs (Gazebo fallback)
- ❌ Readers must install/learn two simulation platforms
- ❌ Some code examples need dual implementations (Gazebo + Isaac variants)

**Validation**: SC-006 requires "all code examples run successfully" - provide both Gazebo and Isaac versions where needed, with clear GPU requirement labels.

---

### ADR-002: LLM Integration Approach (Flexible API with Local Fallback)

**Decision**: Primary path uses OpenAI API (Whisper, GPT-4) with documented local alternatives (Whisper.cpp, Llama 2) for Module 4.

**Context**:
- Module 4 (VLA) requires speech recognition + LLM task planning
- Some readers may prefer local models (privacy, cost, learning)
- OpenAI API is simpler for beginners but requires internet + API key

**Options Considered**:
1. **OpenAI API only**: Simplest, but excludes offline use cases
2. **Local models only**: Privacy-friendly, but complex setup
3. **Flexible (CHOSEN)**: OpenAI primary, local as documented alternative
4. **Cloud-only (Anthropic/OpenAI/etc.)**: Vendor lock-in risk

**Rationale**:
- OpenAI API examples are clearest for beginners (fewer dependencies)
- Local models (Whisper.cpp, Llama 2) documented in sidebars/callouts
- Free tier sufficient for book exercises (SC-014: "free tier sufficient")
- Readers can substitute local models using same ROS 2 interfaces

**Trade-offs**:
- ✅ Beginner-friendly (API setup simpler than model downloads)
- ✅ Flexibility (readers choose privacy vs simplicity)
- ✅ Future-proof (abstractions allow swapping LLM backends)
- ❌ Requires API key setup (friction point)
- ❌ Additional documentation burden (maintain two paths)

**Implementation**:
- Chapter 13-14: Show OpenAI Whisper API + GPT-4 examples
- Sidebar notes: "Local alternative: Use Whisper.cpp + Llama 2 - see Appendix"
- Appendix: Full local model setup guide with code equivalents

**Validation**: FR-008 requires "OpenAI Whisper" and "LLMs as task planners" - satisfied by API approach with local fallback documented.

---

### ADR-003: Chapter Depth vs Breadth Strategy (Depth-First Progressive)

**Decision**: Each chapter goes **deep on 1-2 core concepts** with 3-4 tested examples, rather than broad surveys. Concepts build cumulatively (Chapter N assumes N-1 mastery).

**Context**:
- 16 chapters total, ~2-4 hours per chapter (spec assumption)
- Target: Beginner AI practitioners (know Python/ML, not robotics)
- Constitution requires: 3-5 learning objectives, 3-4 code examples per chapter

**Options Considered**:
1. **Breadth-first**: Survey many topics shallowly (encyclopedia style)
2. **Depth-first (CHOSEN)**: Master 1-2 concepts deeply per chapter
3. **Hybrid**: Mix survey chapters + deep-dive chapters
4. **Project-based**: One continuous project across all chapters

**Rationale**:
- Depth-first aligns with spec: "hands-on coding exercises" and "testable code"
- Shallow surveys don't enable Chapter 16 capstone (needs cumulative skills)
- 3-4 code examples per chapter sufficient for depth (FR-003)
- Progressive difficulty (FR-004): Module 1 foundation → Module 4 integration

**Example Chapter Depth**:
- Chapter 2 (ROS 2 Architecture): Deep dive on pub/sub + services (not launch files, bags, params, etc.)
- Chapter 10 (Perception): Deep dive on YOLO object detection (not segmentation, tracking, etc.)
- Breadth topics deferred to "Further Reading" sections

**Trade-offs**:
- ✅ Readers achieve working mastery (can build things)
- ✅ Capstone project feasible (skills compound)
- ✅ Constitution-compliant (3-4 examples = depth not breadth)
- ❌ Some topics excluded (but listed in "Out of Scope")
- ❌ Readers wanting breadth must supplement with external resources

**Validation**: FR-003 "3-4 tested code examples" and SC-016 "successful capstone completion" require depth over breadth.

---

### ADR-004: Interactive Elements Strategy (Exercises + Optional Extensions)

**Decision**: Each chapter has 1 **required exercise** (clear acceptance criteria) + **optional extension challenges** (labeled difficulty: Beginner/Intermediate/Advanced).

**Context**:
- Spec requires: "one practical exercise with clear acceptance criteria" (FR-003)
- Edge case: "exercises scale for different learning speeds" (Edge Cases section)
- Readers range from cautious beginners to ambitious learners

**Options Considered**:
1. **Single required exercise only**: Uniform experience, but no challenge for fast learners
2. **Multiple required exercises**: Overwhelming for beginners
3. **Tiered exercises (CHOSEN)**: 1 required + optional extensions
4. **Choose-your-own difficulty**: Ambiguous, hard to grade

**Rationale**:
- 1 required exercise = clear success path for all readers
- Extensions provide challenge without penalty (optional)
- Difficulty labels set expectations (Beginner/Intermediate/Advanced)
- Aligns with edge case: "Extension challenges for advanced learners"

**Example (Chapter 2: ROS 2 Architecture)**:
- **Required Exercise**: Build 2-node sensor fusion system (2 publishers, 1 subscriber)
- **Extension - Beginner**: Add a 3rd sensor and visualize in rqt_graph
- **Extension - Intermediate**: Implement timestamped message synchronization
- **Extension - Advanced**: Add service-based reconfiguration for fusion weights

**Trade-offs**:
- ✅ Inclusive (beginners not overwhelmed)
- ✅ Engaging (fast learners stay challenged)
- ✅ Clear acceptance criteria (required exercise graded, extensions optional)
- ❌ More content to write (3-4 exercises per chapter vs 1)
- ❌ Solution appendix larger (must provide solutions for extensions)

**Validation**: FR-003 "one practical exercise with clear acceptance criteria" (required exercise) + Edge Case "Extension challenges for advanced learners" (extensions).

---

### ADR-005: Content Progression Map (Linear Sequential with Feedback Loops)

**Decision**: Modules develop **sequentially** (Module 1 → 2 → 3 → 4), but chapters within a module iterate with **reader feedback loops** before finalizing.

**Context**:
- User guidance: "Research-concurrent approach (iterate chapters with feedback)"
- Module dependencies: Can't teach Nav2 (Module 3) without ROS 2 (Module 1)
- Chapter dependencies: Within a module, chapters can iterate in parallel

**Progression Map**:

```
Phase 0: Setup (Week 1)
├── Docusaurus site scaffold
├── Custom theme for code examples
├── Chapter template with quality gates
└── CI/CD pipeline (build, test, deploy)

Phase 1: Module 1 - Robotic Nervous System (Weeks 2-5)
├── Ch 1: Welcome to Physical AI (Week 2)
│   ├── Draft → Test code → Review → Refine
│   └── Feedback loop: Share with beta readers
├── Ch 2: ROS 2 Architecture (Week 3)
│   └── (same pattern)
├── Ch 3: Describing Robots (URDF) (Week 4)
│   └── (same pattern)
└── Ch 4: Python to ROS (rclpy) (Week 5)
    └── (same pattern + Module 1 integration test)

Phase 2: Module 2 - Digital Twin (Weeks 6-9)
├── Ch 5-8: (same iterative pattern)
└── Dependency: Readers must complete Module 1 first

Phase 3: Module 3 - AI-Robot Brain (Weeks 10-13)
├── Ch 9-12: (same pattern, introduces Isaac Sim)
└── Dependency: Readers must complete Modules 1-2

Phase 4: Module 4 - VLA (Weeks 14-17)
├── Ch 13-16: (same pattern, introduces LLMs)
└── Ch 16: Capstone integrates all previous modules

Phase 5: Polish (Weeks 18-20)
├── Cross-module consistency check
├── Full readability pass (Flesch-Kincaid validation)
├── Plagiarism check (Copyscape)
├── Final beta reader review
└── Production deployment
```

**Feedback Loop Strategy**:
- Draft chapter → Share with 3-5 beta readers (AI practitioners)
- Collect feedback on clarity, exercise difficulty, code runability
- Refine based on feedback before finalizing
- Each module ends with integration test (readers test full module exercises)

**Trade-offs**:
- ✅ Sequential modules prevent dependency confusion
- ✅ Feedback loops improve quality before wide release
- ✅ Iterative refinement aligns with "research-concurrent" guidance
- ❌ Slower development (feedback adds 1-2 weeks per module)
- ❌ Requires beta reader recruitment and management

**Validation**: User guidance "Research-concurrent approach (iterate chapters with feedback)" explicitly requests this pattern.

---

## Testing Strategy

### Quality Gates (Automated - Run on Every Commit)

#### 1. Readability Validation (Constitution Section V)

**Tool**: `textstat` (Python) or Hemingway Editor API
**Metric**: Flesch-Kincaid grade level 10-12
**Threshold**: All chapters must score 10.0-12.0
**Frequency**: Pre-commit hook + CI/CD

**Implementation**:
```bash
# scripts/validate-readability.js
for chapter in docs/**/*.md; do
  score=$(textstat $chapter --flesch-kincaid-grade)
  if [[ $score < 10.0 || $score > 12.0 ]]; then
    echo "FAIL: $chapter scored $score (must be 10-12)"
    exit 1
  fi
done
```

**Acceptance**: SC-004 "100% of written content achieves Flesch-Kincaid grade level 10-12"

---

#### 2. Active Voice & Sentence Length (Constitution Section V)

**Tool**: Grammar analyzer (LanguageTool API or custom regex)
**Metrics**:
- Active voice: ≥75%
- Sentence length: ≤25 words average per section

**Threshold**: FAIL if any chapter violates either metric
**Frequency**: Pre-commit hook + CI/CD

**Implementation**:
```bash
# scripts/validate-grammar.sh
for chapter in docs/**/*.md; do
  active_voice=$(analyze_active_voice $chapter)
  avg_sentence=$(analyze_sentence_length $chapter)

  if [[ $active_voice < 75 ]]; then
    echo "FAIL: $chapter has $active_voice% active voice (must be ≥75%)"
    exit 1
  fi

  if [[ $avg_sentence > 25 ]]; then
    echo "FAIL: $chapter has $avg_sentence words/sentence avg (must be ≤25)"
    exit 1
  fi
done
```

**Acceptance**: SC-005 "100% of content meets 75%+ active voice and 25 words/sentence average"

---

#### 3. Code Example Execution (Constitution Section IV)

**Tool**: pytest + custom test runner
**Metric**: All code examples must execute successfully (exit code 0)
**Threshold**: Zero failures
**Frequency**: Pre-commit hook + CI/CD

**Implementation**:
```bash
# scripts/validate-code-examples.sh
# Extract code blocks from markdown, execute in isolated environments
for chapter in docs/**/*.md; do
  extract_code_blocks $chapter > /tmp/chapter_code.py

  # Test in Docker container with ROS 2 + Gazebo + Isaac Sim
  docker run --rm robotics-env python /tmp/chapter_code.py

  if [[ $? != 0 ]]; then
    echo "FAIL: Code in $chapter failed execution"
    exit 1
  fi
done
```

**Acceptance**: SC-006 "All code examples run successfully in specified environments with zero errors"

---

#### 4. Build Success (Constitution Section VI)

**Tool**: `npm run build`
**Metric**: Build completes with zero errors, zero warnings
**Threshold**: <2 minutes build time
**Frequency**: Every commit (CI/CD)

**Implementation**:
```yaml
# .github/workflows/build-deploy.yml
- name: Build site
  run: |
    start_time=$(date +%s)
    npm run build
    end_time=$(date +%s)
    duration=$((end_time - start_time))

    if [[ $duration > 120 ]]; then
      echo "FAIL: Build took ${duration}s (must be <120s)"
      exit 1
    fi
```

**Acceptance**: SC-009 "Automated build process completes with zero errors and zero warnings in under 2 minutes"

---

#### 5. Link Validation (Constitution Section VI)

**Tool**: `broken-link-checker`
**Metrics**:
- Internal links: Zero 404s
- External links: <5% broken (with documented exceptions)

**Threshold**: FAIL if any internal link broken
**Frequency**: Pre-commit (internal only) + Weekly cron (internal + external)

**Implementation**:
```bash
# scripts/check-links.sh
blc http://localhost:3000 --recursive --filter-level 3 \
  --exclude "*/node_modules/*" \
  --get \
  | tee link-report.txt

internal_broken=$(grep "BROKEN" link-report.txt | grep -v "http" | wc -l)
external_broken=$(grep "BROKEN" link-report.txt | grep "http" | wc -l)
external_total=$(grep "http" link-report.txt | wc -l)

if [[ $internal_broken > 0 ]]; then
  echo "FAIL: $internal_broken internal links broken (must be 0)"
  exit 1
fi

external_rate=$(awk "BEGIN {print ($external_broken / $external_total) * 100}")
if (( $(echo "$external_rate > 5" | bc -l) )); then
  echo "FAIL: $external_rate% external links broken (must be <5%)"
  exit 1
fi
```

**Acceptance**: SC-010 "Zero broken internal links and <5% broken external links"

---

#### 6. Image Optimization (Constitution Section VI)

**Tool**: ImageMagick + file size checker
**Metrics**:
- Format: PNG or SVG only
- Size: ≤500KB per file

**Threshold**: FAIL if any image violates format or size
**Frequency**: Pre-commit hook

**Implementation**:
```bash
# scripts/validate-images.sh
for img in static/img/**/*; do
  format=$(identify -format "%m" $img)
  size=$(stat -f%z $img)

  if [[ $format != "PNG" && $format != "SVG" ]]; then
    echo "FAIL: $img is $format (must be PNG or SVG)"
    exit 1
  fi

  if [[ $size > 512000 ]]; then  # 500KB = 512000 bytes
    echo "FAIL: $img is ${size} bytes (must be ≤500KB)"
    exit 1
  fi
done
```

**Acceptance**: SC-011 "All images render without errors, are optimized to <500KB each"

---

#### 7. Plagiarism Check (Constitution Section VII)

**Tool**: Copyscape API or Turnitin API
**Metric**: <15% similarity (excluding properly cited quotes)
**Threshold**: FAIL if any chapter exceeds 15%
**Frequency**: Pre-release (expensive API, not every commit)

**Implementation**:
```bash
# scripts/check-plagiarism.js
for chapter in docs/**/*.md; do
  similarity=$(copyscape_api_check $chapter)

  if [[ $similarity > 15 ]]; then
    echo "FAIL: $chapter has $similarity% similarity (must be <15%)"
    exit 1
  fi
done
```

**Acceptance**: SC-008 "Content passes plagiarism check with <15% similarity score"

---

#### 8. MCP Server Verification (Constitution Section III)

**Tool**: MCP server contacx7
**Metric**: All technical claims verified as current
**Threshold**: Manual verification before publication
**Frequency**: Before each module release

**Implementation**:
```bash
# Manual workflow (documented in content creation process)
# 1. Draft chapter with citations
# 2. Run MCP server contacx7 to verify each technical claim
# 3. Update citations if sources outdated
# 4. Document verification in chapter front matter:
#    verified_date: 2025-12-05
#    mcp_server: contacx7
```

**Acceptance**: SC-018 "All information is verified as current using MCP server contacx7 before publication"

---

#### 9. Accessibility Check (Constitution Section VI)

**Tool**: `pa11y` (automated WCAG validator)
**Metric**: WCAG 2.1 AA compliance
**Threshold**: Zero violations
**Frequency**: Pre-release (full site audit)

**Implementation**:
```bash
# scripts/check-accessibility.sh
pa11y-ci --sitemap http://localhost:3000/sitemap.xml \
  --standard WCAG2AA \
  --threshold 0  # Zero violations allowed
```

**Acceptance**: SC-011 "meet WCAG 2.1 AA accessibility standards"

---

### Manual Quality Gates (Human Review)

#### 10. Technical Accuracy Review (Constitution Section VI)

**Reviewers**: 2+ robotics practitioners
**Scope**:
- Verify all technical claims (robot kinematics, ROS 2 architecture, physics simulation, AI/ML concepts)
- Test all code examples in clean environment
- Validate exercise solutions

**Threshold**: Zero factual errors
**Frequency**: After each module completion (before publishing)

**Process**:
1. Recruit 2 reviewers with robotics + AI backgrounds
2. Provide module chapters + code examples
3. Reviewers independently test all code in Ubuntu 22.04 + ROS 2 Humble + Gazebo/Isaac
4. Reviewers document any errors/ambiguities
5. Author addresses all feedback before finalizing

**Acceptance**: SC-019 "Technical review by 2+ robotics practitioners confirms zero factual errors"

---

#### 11. Reader Comprehension Testing (Success Metrics)

**Participants**: 5-10 beta readers matching target audience (AI practitioners, no robotics background)
**Scope**:
- Complete Module 1 exercises (test time: 2 weeks)
- Run all code examples (test success rate)
- Complete comprehension quiz (test understanding)

**Metrics**:
- SC-014: Can complete Module 1 exercises within 2 weeks
- SC-015: 90% successfully run all code examples without modification
- SC-017: 80%+ can correctly explain key concepts (ROS 2 pub/sub, URDF structure, SLAM, VLA architecture)

**Threshold**: FAIL if <90% code success rate or <80% comprehension
**Frequency**: After Module 1 completion (pilot test), after full book completion (final validation)

**Process**:
1. Recruit beta readers via AI/ML communities (Reddit, Discord, Twitter)
2. Provide Module 1 chapters with exercises
3. Collect feedback: time to complete, code execution success, comprehension quiz scores
4. Refine content based on feedback
5. Repeat for remaining modules

**Acceptance**: SC-014, SC-015, SC-017 (reader success metrics)

---

### Testing Summary Table

| Gate | Tool | Metric | Threshold | Frequency | Spec Ref |
|------|------|--------|-----------|-----------|----------|
| Readability | textstat | Flesch-Kincaid | 10.0-12.0 | Every commit | SC-004 |
| Grammar | LanguageTool | Active voice | ≥75% | Every commit | SC-005 |
| Grammar | LanguageTool | Sentence length | ≤25 words avg | Every commit | SC-005 |
| Code execution | pytest | Exit code | 0 (success) | Every commit | SC-006 |
| Build | npm | Build time | <2 minutes | Every commit | SC-009 |
| Build | npm | Errors/warnings | 0 | Every commit | SC-009 |
| Links (internal) | broken-link-checker | 404 count | 0 | Every commit | SC-010 |
| Links (external) | broken-link-checker | Broken rate | <5% | Weekly | SC-010 |
| Images (format) | ImageMagick | Format | PNG/SVG only | Pre-commit | SC-011 |
| Images (size) | stat | File size | ≤500KB | Pre-commit | SC-011 |
| Plagiarism | Copyscape | Similarity | <15% | Pre-release | SC-008 |
| MCP verification | contacx7 | Currency | Current | Pre-module | SC-018 |
| Accessibility | pa11y | WCAG 2.1 AA | 0 violations | Pre-release | SC-011 |
| Technical accuracy | Human review | Factual errors | 0 | Post-module | SC-019 |
| Reader success | Beta testing | Code success rate | ≥90% | Post-module | SC-015 |
| Reader success | Beta testing | Comprehension | ≥80% | Post-module | SC-017 |

---

## Implementation Phases

### Phase 0: Infrastructure Setup (Week 1)

**Goal**: Scaffold project infrastructure and establish quality gates.

**Tasks**:
1. Initialize Docusaurus site
   ```bash
   npx create-docusaurus@latest physical-ai-book classic --typescript
   cd physical-ai-book
   npm install
   ```

2. Configure project structure
   - Create `docs/module-01/` through `docs/module-04/` directories
   - Create `static/img/module-XX/chapter-YY/` directories
   - Configure `sidebars.js` with 4-module × 4-chapter structure

3. Set up custom Docusaurus theme
   - Code example syntax highlighting (Python, Bash, YAML)
   - Chapter navigation (prev/next with module context)
   - Progress tracking (chapter completion checkmarks)
   - Difficulty labels for exercises (Beginner/Intermediate/Advanced)

4. Create chapter template (`templates/chapter-template.md`)
   ```markdown
   # Chapter X: [Title]

   ## Overview
   [Brief introduction, 2-3 paragraphs]

   ### Learning Objectives
   - [Objective 1]
   - [Objective 2]
   - [Objective 3]

   ### Prerequisites
   - [What from previous chapters is needed]

   ## Concepts
   [Core explanations with diagrams]

   ## Examples
   ### Example 1: [Title]
   [Code walkthrough]

   ## Exercise
   [Hands-on task with acceptance criteria]

   ## References
   - [Official docs]
   - [Further reading]
   ```

5. Implement quality gates (CI/CD)
   - `.github/workflows/build-deploy.yml` (build + deploy to GitHub Pages)
   - `.github/workflows/quality-check.yml` (readability, grammar, code tests)
   - `.github/workflows/link-checker.yml` (weekly link validation)
   - Pre-commit hooks (image size, readability, code format)

6. Create validation scripts
   - `scripts/validate-readability.js`
   - `scripts/validate-grammar.sh`
   - `scripts/validate-code-examples.sh`
   - `scripts/check-links.sh`
   - `scripts/validate-images.sh`
   - `scripts/check-plagiarism.js`

**Deliverables**:
- Docusaurus site builds successfully
- Custom theme renders correctly
- Chapter template validated
- All quality gates pass (empty content)
- CI/CD pipeline functional

**Acceptance**: Can create a chapter from template, commit, and see all quality gates pass (green CI).

---

### Phase 1: Complete Book Structure with Placeholders (Week 2)

**Goal**: Create all 16 chapter files with placeholder content to establish complete navigation structure and enable early deployment.

**Tasks**:
1. Create placeholder files for all 16 chapters
   - `docs/module-01/chapter-01.md` through `chapter-04.md`
   - `docs/module-02/chapter-05.md` through `chapter-08.md`
   - `docs/module-03/chapter-09.md` through `chapter-12.md`
   - `docs/module-04/chapter-13.md` through `chapter-16.md`

2. Each placeholder contains:
   ```markdown
   # Chapter X: [Title from spec]

   ## Coming Soon

   This chapter is currently being written. It will cover:
   - [Learning objective 1]
   - [Learning objective 2]
   - [Learning objective 3]

   Check back soon for complete content with code examples and exercises.
   ```

3. Create placeholder image directories:
   - `static/img/module-XX/chapter-YY/` for all 16 chapters

4. Test complete build:
   - Verify site builds successfully with all placeholders
   - Verify navigation works across all modules
   - Test deployment to GitHub Pages
   - Verify build time <2 minutes

5. Deploy initial structure:
   - Push to main branch
   - Verify GitHub Pages deployment
   - Confirm all links work, navigation functional

**Deliverables**:
- All 16 chapter files exist with placeholder content
- Complete navigation structure works
- Site deployed to GitHub Pages with placeholder structure
- Build time <2 minutes validated

**Acceptance**: Full book structure is navigable on GitHub Pages with "Coming Soon" placeholders for all chapters.

---

### Phase 2: Module 1 Content - Robotic Nervous System (Weeks 3-6)

**Goal**: Teach ROS 2 foundations (nodes, topics, services, URDF, rclpy). Establish iterative feedback loop pattern.

**Dependencies**: Phase 0 and Phase 1 complete.

**Content Structure** (4 chapters - replace placeholders with full content):

#### Chapter 1: Welcome to Physical AI (Week 2)
- **Overview**: Introduction to embodied intelligence, Physical AI vs Digital AI, book roadmap
- **Concepts**: Sense-think-act loop, why robots need middleware, ROS 2 overview
- **Examples**: Install ROS 2, run "Hello World" node, visualize computation graph
- **Exercise**: Create your first ROS 2 node that publishes "Hello, Physical AI!" messages
- **Acceptance**: Node publishes messages visible in `ros2 topic echo`

#### Chapter 2: ROS 2 Architecture - Nodes, Topics, and Services (Week 3)
- **Overview**: Understanding ROS 2 communication patterns
- **Concepts**: Nodes (independent processes), Topics (pub/sub), Services (request/reply), computation graph
- **Examples**:
  1. Create publisher node (simulated sensor)
  2. Create subscriber node (data consumer)
  3. Visualize with `rqt_graph`
- **Exercise**: Build sensor fusion system (2 sensor nodes publish, 1 fusion node subscribes and combines)
- **Extensions**:
  - Beginner: Add 3rd sensor
  - Intermediate: Implement timestamped message synchronization
  - Advanced: Add service-based reconfiguration
- **Acceptance**: Fusion node correctly combines data from both sensors

#### Chapter 3: Describing Robots - Introduction to URDF (Week 4)
- **Overview**: Mathematical robot descriptions using URDF
- **Concepts**: URDF structure (links, joints, kinematic chains), coordinate frames (TF2), collision vs visual geometry, simplified humanoid structure
- **Examples**:
  1. Dissect 2-link arm URDF
  2. Create basic humanoid torso URDF (fixed joints)
  3. Visualize in RViz2
- **Exercise**: Extend humanoid URDF to add arms with revolute joints
- **Acceptance**: URDF renders in RViz2 with proper hierarchical relationships and no missing transforms

#### Chapter 4: Bridging Python AI to ROS 2 (rclpy) (Week 5)
- **Overview**: Connecting Python AI code to ROS 2 controllers
- **Concepts**: rclpy (Python client library), creating nodes in Python, publishing commands, subscribing to sensor data
- **Examples**:
  1. Write Python node that sends joint commands
  2. Read simulated LiDAR data in Python
  3. Simple reactive behavior: turn away from obstacles
- **Exercise**: Create Python AI agent that reads sensor data and publishes movement commands to avoid obstacles
- **Acceptance**: Robot successfully avoids obstacles in simulation

**Iterative Process** (per chapter):
1. **Draft** (2-3 days): Write content following chapter template
2. **Code** (1 day): Implement and test all code examples
3. **Verify** (1 day): Run MCP server contacx7 on all technical claims, update citations
4. **Review** (1 day): Share with 3-5 beta readers (AI practitioners)
5. **Refine** (1-2 days): Address feedback, fix issues
6. **Quality gates** (automated): Commit triggers readability, code, build checks
7. **Finalize** (1 day): Technical accuracy review, plagiarism check

**Module 1 Integration Test**:
- Beta readers complete all 4 chapter exercises sequentially
- Measure time to complete (target: <2 weeks per SC-014)
- Measure code success rate (target: ≥90% per SC-015)
- Administer comprehension quiz on ROS 2 concepts (target: ≥80% per SC-017)

**Deliverables**:
- 4 chapters published (Ch 1-4)
- 12-16 tested code examples
- 4 exercises with solutions
- Beta reader feedback incorporated
- All quality gates pass

**Acceptance**: Beta readers can complete Module 1 exercises within 2 weeks with ≥90% code success rate.

---

### Phase 3: Module 2 Content - Digital Twin (Weeks 7-10)

**Goal**: Teach physics simulation (Gazebo) and sensor simulation. Readers build realistic robot environments.

**Dependencies**: Phase 2 complete (Module 1 content finished).

**Content Structure** (4 chapters):

#### Chapter 5: Physics Simulation Fundamentals (Week 6)
- **Overview**: Why simulate, physics engine basics
- **Concepts**: Rigid bodies, constraints, contact, gravity, friction, Gazebo architecture
- **Examples**: Spawn cube in Gazebo, adjust friction, simulate falling humanoid
- **Exercise**: Create box stacking simulation, tune physics parameters

#### Chapter 6: Building Your First Humanoid Simulation (Week 7)
- **Overview**: Import URDF into Gazebo, add actuators
- **Concepts**: Gazebo model plugins, joint controllers (position/velocity/effort), launch files
- **Examples**: Load Chapter 3 humanoid into Gazebo, add joint controllers, command joints via ROS 2 topics
- **Exercise**: Make humanoid wave its arm by publishing joint position commands

#### Chapter 7: Sensor Simulation - Eyes and Ears for Robots (Week 8)
- **Overview**: Simulate cameras, depth sensors, LiDAR, IMUs
- **Concepts**: Camera models, RGB-D for 3D perception, LiDAR for obstacles, IMU for orientation
- **Examples**: Add camera to humanoid's head, visualize in RViz2, add LiDAR and visualize point cloud
- **Exercise**: Create 360° vision system with 4 cameras facing different directions

#### Chapter 8: High-Fidelity Rendering with Unity (Optional) (Week 9)
- **Overview**: Use Unity for photorealistic simulation
- **Concepts**: Unity Robotics Hub, ROS-TCP connector, importing URDF, realistic lighting
- **Examples**: Set up Unity with ROS 2 bridge, import humanoid, add materials
- **Exercise**: Create home environment scene in Unity with furniture and obstacles

**Same iterative process as Phase 1.**

**Deliverables**:
- 4 chapters published (Ch 5-8)
- 12-16 tested code examples
- 4 exercises with solutions
- Beta reader feedback incorporated

**Acceptance**: Beta readers can complete Module 2 exercises, observe realistic physics in Gazebo.

---

### Phase 4: Module 3 Content - AI-Robot Brain (Weeks 11-14)

**Goal**: Teach advanced perception (Isaac Sim), computer vision, SLAM, navigation. Transition from Gazebo to Isaac.

**Dependencies**: Phase 3 complete (Modules 1-2 content finished).

**Content Structure** (4 chapters):

#### Chapter 9: Introduction to NVIDIA Isaac Sim (Week 10)
- **Overview**: Isaac Sim vs Gazebo, photorealistic rendering, synthetic data generation
- **Concepts**: RTX rendering, ML data generation, ROS 2 integration
- **Examples**: Install Isaac Sim, load warehouse environment, spawn humanoid
- **Exercise**: Generate 1000 synthetic images of objects in different lighting

#### Chapter 10: Perception Pipelines - Object Detection (Week 11)
- **Overview**: Build computer vision pipelines for robots
- **Concepts**: Object detection models (YOLO), running inference, ROS 2 image transport, bounding boxes to 3D poses
- **Examples**: Run YOLO on humanoid camera feed, detect objects in Isaac Sim, publish as ROS 2 messages
- **Exercise**: Create "fetch the cup" behavior that detects cup and moves toward it

#### Chapter 11: Visual SLAM and Localization (Isaac ROS) (Week 12)
- **Overview**: Hardware-accelerated VSLAM with Isaac ROS
- **Concepts**: SLAM (Simultaneous Localization and Mapping), Visual SLAM vs LiDAR SLAM, Isaac ROS pipelines, map representation
- **Examples**: Run Isaac ROS VSLAM node, build map of environment, visualize pose estimates
- **Exercise**: Have humanoid explore room and build complete map

#### Chapter 12: Path Planning for Bipedal Robots (Nav2) (Week 13)
- **Overview**: Collision-free path planning with Nav2
- **Concepts**: Motion planning (A*, Dijkstra), Nav2 architecture, adapting for bipedal robots, behavior trees
- **Examples**: Set up Nav2 for humanoid, plan path A→B avoiding obstacles, execute path
- **Exercise**: Create patrol behavior where humanoid visits 3 waypoints in sequence

**Same iterative process as Phase 1.**

**Deliverables**:
- 4 chapters published (Ch 9-12)
- 12-16 tested code examples (Gazebo + Isaac versions where needed)
- 4 exercises with solutions
- GPU requirement labels clear (with Gazebo fallbacks)

**Acceptance**: Beta readers can complete Module 3 exercises using Isaac Sim (or Gazebo fallback).

---

### Phase 5: Module 4 Content - Vision-Language-Action (Weeks 15-18)

**Goal**: Teach LLM integration for robotics, voice control, multimodal VLA systems. Culminate in capstone project.

**Dependencies**: Phase 4 complete (Modules 1-3 content finished).

**Content Structure** (4 chapters):

#### Chapter 13: Voice Commands with Whisper (Week 14)
- **Overview**: Enable voice control with OpenAI Whisper
- **Concepts**: Speech-to-text for robotics, Whisper architecture, ROS 2 integration, handling noisy audio
- **Examples**: Install Whisper, capture audio in simulation, transcribe "Go to the kitchen"
- **Exercise**: Create voice-controlled teleop system for humanoid

#### Chapter 14: LLMs as Robot Task Planners (Week 15)
- **Overview**: Use LLMs to translate natural language into robot actions
- **Concepts**: LLMs for cognitive planning, prompt engineering for robotics, breaking "Clean the room" into atomic actions, grounding language to ROS 2 actions
- **Examples**: Use GPT-4 to generate action sequences, map language to ROS 2 service calls, handle failure and re-planning
- **Exercise**: Create system where "Bring me the red cup" generates plan: [detect cup] → [navigate] → [grasp] → [return]

#### Chapter 15: Multimodal Integration - Vision + Language + Action (Week 16)
- **Overview**: Combine vision, language, and action into unified VLA system
- **Concepts**: VLA architecture (RT-1/RT-2 style), connecting perception/planning/control, closed-loop execution, failure recovery
- **Examples**: Integrate Whisper + LLM + Nav2 + Object Detection, execute "Find the door and open it", handle errors
- **Exercise**: Build "find and fetch" system that takes voice commands and executes end-to-end

#### Chapter 16: Capstone - The Autonomous Humanoid Butler (Week 17)
- **Overview**: Integrate everything into fully autonomous humanoid system
- **Concepts**: System integration best practices, debugging complex systems, performance optimization, sim-to-real transfer (future directions)
- **Capstone Project**: Build simulated humanoid butler that:
  1. Listens for voice commands ("Bring me a snack")
  2. Plans action sequence using LLM
  3. Navigates to kitchen using Nav2
  4. Detects and identifies snacks using computer vision
  5. Returns to human and reports completion
- **Extensions**: Multiple rooms, multiple objects, obstacle handling
- **Exercise**: Extend butler with additional capabilities per spec

**Same iterative process as Phase 1, with extra emphasis on capstone integration testing.**

**Deliverables**:
- 4 chapters published (Ch 13-16)
- 12-16 tested code examples (OpenAI API + local alternatives documented)
- 4 exercises with solutions
- Capstone project fully functional

**Acceptance**: Beta readers successfully complete Chapter 16 capstone project demonstrating end-to-end autonomy.

---

### Phase 6: Polish & Final Validation (Weeks 19-20)

**Goal**: Final quality pass, consistency check, full validation against all success criteria.

**Tasks**:

1. **Cross-Module Consistency Check** (Week 18)
   - Terminology audit (ensure glossary terms used consistently)
   - Code style audit (ensure all examples follow same conventions)
   - Navigation audit (ensure cross-references work, sidebar coherent)
   - Voice/tone audit (ensure uniform throughout)

2. **Full Quality Validation** (Week 19)
   - Run all automated quality gates on complete book
   - Readability: textstat on all 16 chapters (target FK 10-12)
   - Grammar: LanguageTool on all 16 chapters (≥75% active voice, ≤25 words/sentence)
   - Code: Execute all 48-64 code examples in clean environments
   - Build: Verify build time <2 minutes
   - Links: Run broken-link-checker (zero internal 404s)
   - Images: Validate all images <500KB, PNG/SVG format
   - Plagiarism: Run Copyscape on all chapters (<15% similarity)
   - Accessibility: Run pa11y (WCAG 2.1 AA compliance)

3. **Final Beta Reader Review** (Week 19)
   - Recruit 10 fresh beta readers (haven't seen earlier drafts)
   - Provide full book (all 16 chapters)
   - Collect metrics:
     - Time to complete exercises (target: 8-12 weeks)
     - Code success rate (target: ≥90%)
     - Capstone completion (target: 100%)
     - Comprehension quiz (target: ≥80%)
   - Address any critical feedback

4. **Technical Accuracy Final Review** (Week 20)
   - 2 robotics practitioners review full book
   - Verify zero factual errors (kinematics, ROS 2, physics, AI/ML)
   - Test all code in Ubuntu 22.04 + ROS 2 Humble + Gazebo + Isaac
   - Sign off on technical accuracy

5. **Production Deployment** (Week 20)
   - Merge to main branch
   - Deploy to GitHub Pages: https://[username].github.io/physical-ai-book
   - Announce to AI/robotics communities
   - Monitor for reader feedback and errata

**Deliverables**:
- All 16 chapters finalized
- All 22 success criteria validated (see Success Criteria Validation table below)
- Book deployed to production
- Errata tracking system in place

**Acceptance**: All success criteria pass (100% compliance).

---

## Success Criteria Validation

| Success Criterion | Validation Method | Expected Result | Phase |
|-------------------|-------------------|-----------------|-------|
| **SC-001**: 16 chapters written, reviewed, published | Manual count + git log | 16 chapters in `docs/` | Phase 6 |
| **SC-002**: Each chapter has 4 sections | Script: check section headings | All chapters have Overview, Concepts, Examples, References | Phase 6 |
| **SC-003**: 100% chapters have ≥3 code examples + 1 exercise | Script: count code blocks + exercises | All pass | Phase 6 |
| **SC-004**: Flesch-Kincaid 10-12 | textstat on all chapters | All chapters score 10.0-12.0 | Phase 6 |
| **SC-005**: 75%+ active voice, ≤25 words/sentence | LanguageTool on all chapters | All chapters pass both metrics | Phase 6 |
| **SC-006**: All code runs successfully | Execute all code examples in Docker | Exit code 0 for all | Phase 6 |
| **SC-007**: 100% technical claims cited | Script: check for citation links | All technical statements have `[source]` links | Phase 6 |
| **SC-008**: <15% plagiarism | Copyscape API on all chapters | All chapters <15% similarity | Phase 6 |
| **SC-009**: Build <2 min, zero errors/warnings | CI/CD build metrics | Build time <120s, exit code 0 | Every commit |
| **SC-010**: Zero internal 404s, <5% external broken | broken-link-checker | Internal: 0 broken, External: <5% | Weekly |
| **SC-011**: Images render correctly, <500KB, WCAG AA | Image validator + pa11y | All images pass | Phase 6 |
| **SC-012**: Deployment succeeds automatically | GitHub Actions status | Green checkmark on deploy workflow | Every merge |
| **SC-013**: Navigation: 100% links valid, state persists | Manual testing + link checker | All sidebar/cross-refs work | Phase 1 |
| **SC-014**: Readers complete Module 1 in 2 weeks | Beta reader survey | ≥80% complete within 14 days | Phase 2 |
| **SC-015**: 90% readers run code successfully | Beta reader survey | ≥90% report success | Phase 6 |
| **SC-016**: Readers complete Chapter 16 capstone | Beta reader survey | 100% completion rate | Phase 5 |
| **SC-017**: 80%+ comprehension quiz | Administer quiz to beta readers | ≥80% correct answers | Phase 6 |
| **SC-018**: MCP server contacx7 verification | Manual checklist | All chapters verified | Each module |
| **SC-019**: 2+ reviewers confirm zero factual errors | Technical review sign-off | Zero errors reported | Phase 6 |
| **SC-020**: Software versions tested and compatible | Docker environment validation | All code runs in Ubuntu 22.04 + ROS 2 Humble + Gazebo + Isaac | Phase 6 |
| **SC-021**: Capstone integrates all 15 chapters | Code review of Chapter 16 | Uses ROS 2 + URDF + Gazebo/Isaac + sensors + perception + SLAM + Nav2 + Whisper + LLM | Phase 5 |
| **SC-022**: Capstone demonstrates autonomous behavior | Video demo + code execution | Voice → task → navigation → detection → manipulation → reporting | Phase 5 |

---

## Risk Mitigation

| Risk | Likelihood | Impact | Mitigation Strategy |
|------|------------|--------|---------------------|
| **Beta readers unavailable** | Medium | High | Recruit from multiple communities (Reddit, Discord, Twitter), offer early access incentive |
| **Isaac Sim GPU requirements exclude readers** | High | Medium | Provide Gazebo alternatives for all Module 3-4 exercises, document cloud GPU options |
| **Code examples break with software updates** | Medium | High | Pin exact versions (ROS 2 Humble, Isaac 2023.1+), document in prerequisites, test in Docker |
| **Readability scores fail** | Low | Medium | Iterative refinement with feedback, use Hemingway Editor during drafting |
| **Plagiarism check fails** | Low | High | Original writing, cite all sources, run Copyscape early and often |
| **Build time exceeds 2 minutes** | Low | Low | Optimize images, minimize dependencies, use incremental builds |
| **Capstone too complex for beginners** | Medium | High | Provide scaffolded starter code, break into smaller milestones, offer simplified version |
| **LLM API costs for readers** | Medium | Medium | Emphasize free tier sufficiency, document local model alternatives (Whisper.cpp, Llama 2) |
| **Scope creep (too many topics)** | High | High | Strictly enforce "Out of Scope" list, depth-first approach, defer breadth topics to "Further Reading" |
| **Timeline delays** | High | Medium | Build buffer into each phase (1-2 weeks), prioritize critical path (Modules 1-3), consider Module 4 as stretch goal |

---

## Next Steps

1. **Immediate**: Begin Phase 0 (Infrastructure Setup)
   - Initialize Docusaurus site
   - Configure custom theme
   - Set up CI/CD pipelines
   - Create chapter template

2. **Week 2**: Complete Phase 1 (Book Structure)
   - Create all 16 placeholder chapter files
   - Test navigation across all modules
   - Deploy initial structure to GitHub Pages
   - Verify build and deployment pipeline

3. **Week 3**: Start Phase 2 (Module 1 Content)
   - Replace Chapter 1 placeholder with full content
   - Progress through Chapters 2-4
   - Recruit beta readers
   - Begin iterative feedback loop

4. **Ongoing**:
   - Weekly progress reviews
   - Monthly constitution compliance audits
   - Progressive deployment (deploy each module as completed)

5. **Completion Target**: Week 20 (5 months from start)

**Recommended Next Command**: `/sp.tasks` to generate actionable task breakdown for Phase 0.

---

**Plan Status**: READY FOR IMPLEMENTATION
**Constitution Compliance**: ✅ PASS (all requirements aligned)
**Success Criteria Coverage**: 22/22 criteria addressed
**Risk Level**: MEDIUM (mitigations documented)
