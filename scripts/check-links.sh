#!/bin/bash

##
# Link Checker Script
#
# Validates all links in markdown files
# - Internal links: 0 broken (SC-010)
# - External links: <5% broken (SC-010)
#
# Usage: bash scripts/check-links.sh
##

echo ""
echo "🔗 Link Validation"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "Note: Full link checking runs via GitHub Actions (link-checker.yml)"
echo "   This validates internal markdown links only"
echo ""

TARGET="${1:-docs}"

# Find broken internal links
BROKEN=0

mapfile -t FILES < <(find "$TARGET" -name "*.md" -not -path "*/node_modules/*")

for file in "${FILES[@]}"; do
  # Extract markdown links [text](path)
  while IFS= read -r link; do
    # Skip external links
    if [[ "$link" =~ ^http ]]; then
      continue
    fi

    # Check if file exists
    LINK_PATH=$(dirname "$file")/"$link"
    if [ ! -f "$LINK_PATH" ] && [ ! -f "$LINK_PATH.md" ]; then
      echo "❌ Broken link in $file: $link"
      BROKEN=$((BROKEN + 1))
    fi
  done < <(grep -oP '\]\(\K[^)]+' "$file" 2>/dev/null || true)
done

if [ "$BROKEN" -gt 0 ]; then
  echo ""
  echo "❌ Found $BROKEN broken internal links"
  exit 1
fi

echo "✅ All internal links valid!"
echo ""
exit 0
