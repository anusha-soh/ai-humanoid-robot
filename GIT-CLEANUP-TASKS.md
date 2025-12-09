# Git Branch Cleanup - Task Breakdown

**Project:** Physical AI & Humanoid Robotics Book
**Purpose:** Consolidate 4 divergent git branches into 2 clean branches
**Timeline:** 15-20 minutes
**Risk Level:** Low (all work preserved, recoverable for 30 days)

---

## Task Overview

**Total Tasks:** 15
**Phases:** 4
**Completion Criteria:** 2 clean branches (master + 001-physical-ai-book) with all work preserved

---

## Phase 1: Pre-Cleanup Verification & Backup

**Goal:** Ensure safety before making any changes

- [ ] T001 Verify current branch is 001-physical-ai-book
- [ ] T002 Verify all changes are committed (git status should be clean)
- [ ] T003 Push current branch to remote: git push origin 001-physical-ai-book
- [ ] T004 Create local backup: cp -r humanoid-ai-robot humanoid-ai-robot-backup
- [ ] T005 Document current branch structure: git branch -a > branch-snapshot.txt

**Acceptance:** All local changes committed and pushed, backup created

---

## Phase 2: Extract Deployment Configuration

**Goal:** Preserve Vercel config from improve-constitution-standards branch

- [ ] T006 Checkout Vercel config: git checkout improve-constitution-standards -- vercel.json
- [ ] T007 Checkout GitHub workflows: git checkout improve-constitution-standards -- .github/workflows/
- [ ] T008 Stage deployment files: git add vercel.json .github/
- [ ] T009 Commit deployment config: git commit -m "chore: add Vercel deployment configuration and CI workflows"
- [ ] T010 Push to remote: git push origin 001-physical-ai-book

**Acceptance:** Vercel config and workflows exist in current branch

---

## Phase 3: Update Master Branch with Complete Content

**Goal:** Merge all content from 001-physical-ai-book into master

- [ ] T011 Switch to master branch: git checkout master
- [ ] T012 Merge current branch into master: git merge 001-physical-ai-book --no-ff -m "feat: merge complete book content and UI from 001-physical-ai-book"
- [ ] T013 Push updated master to remote: git push origin master
- [ ] T014 Verify master has full content: git show master:docs/module-01/chapter-01.md | head -10

**Acceptance:** Master branch contains all 16 complete chapters + custom UI + deployment config

---

## Phase 4: Delete Obsolete Branches & Verify

**Goal:** Clean up merged branches and verify final state

- [ ] T015 Switch back to working branch: git checkout 001-physical-ai-book
- [ ] T016 Delete local 002-minimal-ui-theme: git branch -d 002-minimal-ui-theme
- [ ] T017 Delete local improve-constitution-standards: git branch -d improve-constitution-standards
- [ ] T018 Delete remote improve-constitution-standards: git push origin --delete improve-constitution-standards
- [ ] T019 Prune stale remote references: git fetch --prune && git remote prune origin
- [ ] T020 Verify final branch count: git branch -a (should show 2 local, 2 remote)
- [ ] T021 Verify master deployment readiness: Check Vercel dashboard for auto-deploy trigger
- [ ] T022 Test deployed site: Visit Vercel URL and verify all 16 chapters load with full content

**Acceptance:** Only 2 branches remain (master, 001-physical-ai-book), all work preserved, Vercel deploying from master

---

## Dependency Graph

```
Phase 1 (Verification) → Phase 2 (Extract Config) → Phase 3 (Update Master) → Phase 4 (Cleanup)
```

**Sequential Execution Required:** All phases must complete in order.

**No Parallel Opportunities:** Git operations must be sequential to maintain integrity.

---

## Rollback Plan

If anything goes wrong during cleanup:

1. **Stop immediately** and do not proceed with remaining tasks
2. **Restore from backup:**
   ```bash
   cd ..
   rm -rf humanoid-ai-robot
   cp -r humanoid-ai-robot-backup humanoid-ai-robot
   cd humanoid-ai-robot
   ```
3. **Or recover deleted branch from reflog:**
   ```bash
   git reflog  # Find commit SHA of deleted branch
   git checkout -b <branch-name> <commit-sha>
   ```

---

## Post-Cleanup Verification Checklist

After completing all tasks, verify:

- [ ] Master branch has all 16 complete chapters (not placeholders)
- [ ] Master branch has custom UI (Hero, HomepageFeatures)
- [ ] Master branch has vercel.json configuration
- [ ] Master branch has .github/workflows/ directory
- [ ] Only 2 local branches exist: master, 001-physical-ai-book
- [ ] Only 2 remote branches exist: origin/master, origin/001-physical-ai-book
- [ ] Vercel deployment triggered from master branch
- [ ] Deployed site shows full content at all chapter URLs

---

## Success Criteria

✅ **Branch Count:** Reduced from 4 to 2 branches
✅ **Work Preserved:** All content, UI, and config consolidated
✅ **Deployment Ready:** Master branch ready for production deployment
✅ **No Data Loss:** All commits accessible, backup available
✅ **Clean History:** Clear branch structure for future development

---

## Execution Notes

- **Estimated Time:** 15-20 minutes
- **Difficulty:** Easy (copy-paste commands)
- **Risk:** Low (all reversible)
- **Dependencies:** Git, network connection to GitHub
- **Prerequisites:** Clean working directory, all changes committed

---

## Commands Quick Reference

**Check current branch:**
```bash
git branch --show-current
```

**Check git status:**
```bash
git status
```

**View all branches:**
```bash
git branch -a
```

**Check what's in a branch:**
```bash
git show <branch-name>:<file-path> | head -20
```

**Recover deleted branch:**
```bash
git reflog
git checkout -b <branch-name> <commit-sha>
```

---

**Ready to execute?** Start with Phase 1, Task T001.
