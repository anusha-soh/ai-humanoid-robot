# Git Branch Cleanup Plan
**Project:** Physical AI & Humanoid Robotics Book
**Date:** 2024-12-09
**Current Branch:** 001-physical-ai-book
**Status:** Ready for cleanup

---

## Executive Summary

Your repository has **4 local branches** with divergent histories. The current branch `001-physical-ai-book` has the most complete and up-to-date content, but useful deployment configurations exist in other branches. This plan will consolidate everything into 2 clean branches without losing any work.

---

## Current Branch Status

### ✅ **001-physical-ai-book** (CURRENT - MOST COMPLETE)
**Status:** Active development branch with all content
**Contains:**
- ✅ All 16 chapters with full content (avg 1,957 words each)
- ✅ Custom UI theme (Hero, HomepageFeatures)
- ✅ Module icons (SVG files for all 4 modules)
- ✅ 64 code examples
- ✅ Complete navigation structure

**Commits ahead of master:** 2
- `b11d75d` - feat: integrate custom UI theme from master branch
- `b5bc73c` - feat: add complete content for all 16 chapters and fix landing page

**Action:** ✅ **KEEP** - This is your primary working branch

---

### ⚠️ **master** (OUTDATED - NEEDS UPDATE)
**Status:** Deployed to Vercel but contains only placeholder content
**Contains:**
- ❌ Placeholder chapters ("Coming Soon" text)
- ✅ Custom UI theme (merged from 002-minimal-ui-theme)
- ❌ Missing all full chapter content

**Problem:** Vercel is likely deploying from this branch, showing old placeholder content

**Action:** 🔄 **UPDATE** - Merge 001-physical-ai-book into master to deploy full content

---

### ⚠️ **improve-constitution-standards** (HAS DEPLOYMENT CONFIG)
**Status:** Contains Vercel configuration and GitHub workflows
**Contains:**
- ✅ `vercel.json` - Vercel deployment configuration
- ✅ `.github/workflows/deploy.yml` - Deployment workflow
- ✅ `.github/workflows/link-checker.yml` - Link validation
- ✅ `.github/workflows/quality-check.yml` - Quality checks
- ❌ Outdated chapter content (placeholders)

**Unique commits:** 5 deployment-related commits
**Action:** 🔄 **EXTRACT THEN DELETE** - Cherry-pick deployment files, then delete branch

---

### ✅ **002-minimal-ui-theme** (FULLY MERGED)
**Status:** Feature branch fully integrated
**Contains:**
- Custom UI theme (Hero component, features, icons)
- All work already in master and 001-physical-ai-book

**Action:** 🗑️ **DELETE** - Safe to remove, all work preserved

---

## Cleanup Strategy

### Phase 1: Extract Deployment Configuration ⚠️ CRITICAL

1. **Cherry-pick Vercel config from improve-constitution-standards:**
   ```bash
   git checkout 001-physical-ai-book
   git checkout improve-constitution-standards -- vercel.json
   git checkout improve-constitution-standards -- .github/workflows/
   git add vercel.json .github/
   git commit -m "chore: add Vercel deployment configuration and CI workflows"
   git push origin 001-physical-ai-book
   ```

### Phase 2: Update Master Branch with Full Content

2. **Merge all content into master:**
   ```bash
   git checkout master
   git merge 001-physical-ai-book --no-ff -m "feat: merge complete book content and UI from 001-physical-ai-book"
   git push origin master
   ```

   **Result:** Master will now have:
   - All 16 complete chapters
   - Custom UI theme
   - Deployment configuration
   - Ready for Vercel deployment

### Phase 3: Delete Merged Feature Branches

3. **Delete 002-minimal-ui-theme (locally and remotely):**
   ```bash
   # Delete local branch
   git branch -d 002-minimal-ui-theme

   # Note: No remote branch to delete (not pushed to origin)
   ```

4. **Delete improve-constitution-standards (locally and remotely):**
   ```bash
   # Delete local branch
   git branch -d improve-constitution-standards

   # Delete remote branch
   git push origin --delete improve-constitution-standards
   ```

### Phase 4: Verify and Clean Up

5. **Verify remaining branches:**
   ```bash
   git branch -a
   ```

   **Expected output:**
   ```
   * 001-physical-ai-book
     master
     remotes/origin/001-physical-ai-book
     remotes/origin/master
   ```

6. **Prune stale remote references:**
   ```bash
   git fetch --prune
   git remote prune origin
   ```

---

## Final Branch Structure

After cleanup, you'll have exactly **2 branches**:

### 📌 **master** (Main Deployment Branch)
- **Purpose:** Production-ready code deployed to Vercel
- **Contains:** All content + UI + deployment config
- **Protected:** Yes (should be your default branch)

### 📌 **001-physical-ai-book** (Feature/Development Branch)
- **Purpose:** Ongoing book content development
- **Contains:** Same as master (after merge)
- **Usage:** Continue using for new features/chapters

---

## Verification Checklist

After running the cleanup:

- [ ] Master branch has all 16 complete chapters (not placeholders)
- [ ] Master branch has custom UI (Hero, HomepageFeatures)
- [ ] Master branch has `vercel.json` configuration
- [ ] Master branch has `.github/workflows/` directory
- [ ] Vercel deployment is triggered from master
- [ ] Only 2 local branches remain (master, 001-physical-ai-book)
- [ ] Only 2 remote branches remain (origin/master, origin/001-physical-ai-book)
- [ ] All stale remote references are pruned

---

## Deployment Configuration Summary

### Vercel Configuration (`vercel.json`)
```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "framework": "docusaurus",
  "installCommand": "npm install",
  "devCommand": "npm start"
}
```

### Required Environment Variables (Vercel Dashboard)
None required - static Docusaurus site

### Recommended Vercel Settings
- **Production Branch:** master
- **Auto-deploy:** Enabled for master branch
- **Build Command:** `npm run build` (from vercel.json)
- **Output Directory:** `build`
- **Install Command:** `npm install`

---

## Risk Mitigation

### ⚠️ Before Starting Cleanup:

1. **Create a backup:**
   ```bash
   cd ..
   cp -r humanoid-ai-robot humanoid-ai-robot-backup
   ```

2. **Verify current branch is clean:**
   ```bash
   git status
   ```
   All changes should be committed.

3. **Ensure remote is up to date:**
   ```bash
   git push origin 001-physical-ai-book
   ```

### 🆘 Recovery Plan (if something goes wrong):

If you need to restore a deleted branch:
```bash
# Find the commit SHA of the deleted branch
git reflog

# Recreate the branch
git checkout -b <branch-name> <commit-sha>
```

Branches are kept in reflog for ~30 days even after deletion.

---

## Post-Cleanup Tasks

1. **Update Vercel project settings:**
   - Set production branch to `master`
   - Verify build triggers on master push
   - Check deployment logs

2. **Test the deployed site:**
   - Visit your Vercel URL
   - Verify all 16 chapters load with full content
   - Check custom UI renders correctly
   - Test navigation between modules

3. **Update local development workflow:**
   - Continue using `001-physical-ai-book` for new features
   - Merge to `master` when ready to deploy
   - Delete feature branches after merging

4. **Document the new workflow:**
   - Add to README.md or CONTRIBUTING.md
   - Explain branching strategy for future features

---

## Timeline

Estimated time: **15-20 minutes**

- Phase 1 (Extract config): 5 min
- Phase 2 (Update master): 5 min
- Phase 3 (Delete branches): 3 min
- Phase 4 (Verify): 2 min
- Post-cleanup tasks: 5 min

---

## Questions & Answers

**Q: Will I lose any work?**
A: No. All work is preserved in the current branch and will be merged into master. Deleted branches can be recovered from reflog for 30 days.

**Q: What if Vercel is deploying from a different branch?**
A: Check your Vercel project settings after cleanup and set the production branch to `master`.

**Q: Can I keep working while cleanup is in progress?**
A: It's best to complete all cleanup steps before resuming work to avoid conflicts.

**Q: What if the merge to master has conflicts?**
A: Unlikely since master has placeholders and current branch has full content. If conflicts occur, choose the version from `001-physical-ai-book` (current branch).

**Q: Should I delete the backup after cleanup?**
A: Keep it for 1-2 weeks, then delete once you're confident everything works.

---

## Ready to Execute?

✅ **Prerequisites Met:**
- All changes committed to 001-physical-ai-book
- Changes pushed to remote
- Backup created (recommended)

🚀 **Next Step:** Run Phase 1 commands to extract deployment configuration

---

**Last Updated:** 2024-12-09
**Author:** Claude Code (Automated Cleanup Plan)
