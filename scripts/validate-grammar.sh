#!/bin/bash

##
# Grammar Validation Script
#
# Validates content meets grammar and style requirements:
# - Active voice: ≥75% (FR-002)
# - Sentence length: ≤25 words average (FR-002)
# - Constitution requirements
#
# Usage: bash scripts/validate-grammar.sh [file-or-directory]
##

set -e

TARGET="${1:-docs}"
MIN_ACTIVE_VOICE=75
MAX_SENTENCE_LENGTH=25

echo ""
echo "📝 Grammar & Style Validation"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Target: $TARGET"
echo "Active voice requirement: ≥${MIN_ACTIVE_VOICE}%"
echo "Sentence length requirement: ≤${MAX_SENTENCE_LENGTH} words"
echo ""

# Find all markdown files
if [ -f "$TARGET" ]; then
  FILES=("$TARGET")
else
  mapfile -t FILES < <(find "$TARGET" -name "*.md" -not -path "*/node_modules/*" -not -path "*/.docusaurus/*")
fi

echo "Files to check: ${#FILES[@]}"
echo ""

TOTAL_FILES=0
PASSED_FILES=0
FAILED_FILES=0
FAILURES=()

for file in "${FILES[@]}"; do
  # Skip empty files
  if [ ! -s "$file" ]; then
    continue
  fi

  TOTAL_FILES=$((TOTAL_FILES + 1))

  # Strip code blocks and get prose content
  CONTENT=$(sed '/^```/,/^```/d' "$file" | \
            sed 's/`[^`]*`//g' | \
            sed '/^---/,/^---/d' | \
            sed 's/^#.*$//' | \
            tr '\n' ' ')

  # Skip if content too short
  WORD_COUNT=$(echo "$CONTENT" | wc -w)
  if [ "$WORD_COUNT" -lt 50 ]; then
    PASSED_FILES=$((PASSED_FILES + 1))
    continue
  fi

  # Check passive voice indicators
  PASSIVE_COUNT=$(echo "$CONTENT" | grep -oiE '\b(was|were|been|being|is|are|am)\s+\w+ed\b' | wc -l)
  SENTENCE_COUNT=$(echo "$CONTENT" | grep -oE '[.!?]' | wc -l)

  if [ "$SENTENCE_COUNT" -eq 0 ]; then
    SENTENCE_COUNT=1
  fi

  # Estimate active voice percentage
  PASSIVE_RATE=$(awk "BEGIN {print int(($PASSIVE_COUNT / $SENTENCE_COUNT) * 100)}")
  ACTIVE_RATE=$((100 - PASSIVE_RATE))

  # Check average sentence length
  AVG_LENGTH=$(awk "BEGIN {print int($WORD_COUNT / $SENTENCE_COUNT)}")

  # Validate
  FILE_PASSED=true
  REASONS=()

  if [ "$ACTIVE_RATE" -lt "$MIN_ACTIVE_VOICE" ]; then
    FILE_PASSED=false
    REASONS+=("Active voice ${ACTIVE_RATE}% (need ≥${MIN_ACTIVE_VOICE}%)")
  fi

  if [ "$AVG_LENGTH" -gt "$MAX_SENTENCE_LENGTH" ]; then
    FILE_PASSED=false
    REASONS+=("Avg sentence ${AVG_LENGTH} words (need ≤${MAX_SENTENCE_LENGTH})")
  fi

  if [ "$FILE_PASSED" = true ]; then
    PASSED_FILES=$((PASSED_FILES + 1))
  else
    FAILED_FILES=$((FAILED_FILES + 1))
    FAILURE_MSG="  ❌ $file: ${REASONS[*]}"
    FAILURES+=("$FAILURE_MSG")
  fi
done

echo "✅ Passed: $PASSED_FILES"
echo "❌ Failed: $FAILED_FILES"
echo ""

if [ "$FAILED_FILES" -gt 0 ]; then
  echo "Failed Files:"
  for failure in "${FAILURES[@]}"; do
    echo "$failure"
  done
  echo ""
  exit 1
fi

echo "✅ All files pass grammar and style requirements!"
echo ""
exit 0
