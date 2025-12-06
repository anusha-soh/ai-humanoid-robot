#!/usr/bin/env node

/**
 * Readability Validation Script
 *
 * Validates that all markdown content meets Flesch-Kincaid readability requirements:
 * - Target: Grade level 10-12 (FR-001)
 * - Constitution requirement: Flesch-Kincaid 10-12
 *
 * Usage: node scripts/validate-readability.js [file-or-directory]
 */

const fs = require('fs');
const path = require('path');
const { execSync } = require('child_process');

// Flesch-Kincaid grade level thresholds
const MIN_GRADE = 10;
const MAX_GRADE = 12;

/**
 * Calculate Flesch-Kincaid Grade Level
 * Formula: 0.39 * (words/sentences) + 11.8 * (syllables/words) - 15.59
 */
function calculateFleschKincaid(text) {
  const sentences = text.split(/[.!?]+/).filter(s => s.trim().length > 0).length;
  const words = text.split(/\s+/).filter(w => w.trim().length > 0).length;
  const syllables = countSyllables(text);

  if (sentences === 0 || words === 0) return 0;

  const gradeLevel = 0.39 * (words / sentences) + 11.8 * (syllables / words) - 15.59;
  return Math.round(gradeLevel * 10) / 10;
}

/**
 * Count syllables in text (simplified algorithm)
 */
function countSyllables(text) {
  const words = text.toLowerCase().match(/\b[a-z]+\b/g) || [];
  return words.reduce((total, word) => {
    // Simple syllable count: vowel groups
    const syllables = word.match(/[aeiouy]+/g);
    return total + (syllables ? syllables.length : 1);
  }, 0);
}

/**
 * Find all markdown files in directory
 */
function findMarkdownFiles(dir) {
  const files = [];
  const items = fs.readdirSync(dir);

  for (const item of items) {
    const fullPath = path.join(dir, item);
    const stat = fs.statSync(fullPath);

    if (stat.isDirectory() && !item.startsWith('.') && item !== 'node_modules') {
      files.push(...findMarkdownFiles(fullPath));
    } else if (item.endsWith('.md')) {
      files.push(fullPath);
    }
  }

  return files;
}

/**
 * Remove code blocks and technical content for readability analysis
 */
function stripCodeBlocks(content) {
  // Remove code blocks
  content = content.replace(/```[\s\S]*?```/g, '');
  // Remove inline code
  content = content.replace(/`[^`]+`/g, '');
  // Remove links
  content = content.replace(/\[([^\]]+)\]\([^)]+\)/g, '$1');
  // Remove YAML frontmatter
  content = content.replace(/^---[\s\S]*?---/m, '');
  // Remove headings markers
  content = content.replace(/^#+\s+/gm, '');

  return content.trim();
}

/**
 * Validate a single file
 */
function validateFile(filePath) {
  const content = fs.readFileSync(filePath, 'utf8');
  const cleanContent = stripCodeBlocks(content);

  if (cleanContent.length < 100) {
    return { pass: true, grade: 'N/A', reason: 'Content too short to analyze' };
  }

  const grade = calculateFleschKincaid(cleanContent);
  const pass = grade >= MIN_GRADE && grade <= MAX_GRADE;

  return { pass, grade, min: MIN_GRADE, max: MAX_GRADE };
}

/**
 * Main execution
 */
function main() {
  const target = process.argv[2] || 'docs';
  let files = [];

  if (fs.statSync(target).isDirectory()) {
    files = findMarkdownFiles(target);
  } else {
    files = [target];
  }

  console.log(`\n📖 Readability Validation`);
  console.log(`━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n`);
  console.log(`Target: ${target}`);
  console.log(`Files: ${files.length}`);
  console.log(`Grade level requirement: ${MIN_GRADE}-${MAX_GRADE}\n`);

  const results = files.map(file => ({
    file: path.relative(process.cwd(), file),
    ...validateFile(file)
  }));

  const failures = results.filter(r => !r.pass);
  const passed = results.filter(r => r.pass);

  // Print results
  console.log(`✅ Passed: ${passed.length}`);
  console.log(`❌ Failed: ${failures.length}\n`);

  if (failures.length > 0) {
    console.log(`Failed Files:`);
    failures.forEach(r => {
      console.log(`  ❌ ${r.file}: Grade ${r.grade} (expected ${r.min}-${r.max})`);
    });
    console.log();
    process.exit(1);
  }

  console.log(`✅ All files pass readability requirements!\n`);
  process.exit(0);
}

if (require.main === module) {
  main();
}

module.exports = { calculateFleschKincaid, validateFile };
