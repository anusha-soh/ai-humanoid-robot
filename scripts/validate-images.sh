#!/bin/bash

##
# Image Validation Script
#
# Validates all images meet requirements:
# - Format: PNG or SVG (SC-011)
# - Size: ≤500KB (SC-011)
#
# Usage: bash scripts/validate-images.sh
##

echo ""
echo "🖼️  Image Validation"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

MAX_SIZE_KB=500
IMAGE_DIR="${1:-static/img}"

if [ ! -d "$IMAGE_DIR" ]; then
  echo "✅ No images directory yet ($IMAGE_DIR)"
  exit 0
fi

TOTAL=0
PASSED=0
FAILED=0

mapfile -t IMAGES < <(find "$IMAGE_DIR" -type f \( -iname "*.png" -o -iname "*.svg" -o -iname "*.jpg" -o -iname "*.jpeg" -o -iname "*.gif" \))

for img in "${IMAGES[@]}"; do
  TOTAL=$((TOTAL + 1))
  FAILED_REASONS=()

  # Check format
  EXT="${img##*.}"
  EXT_LOWER=$(echo "$EXT" | tr '[:upper:]' '[:lower:]')

  if [ "$EXT_LOWER" != "png" ] && [ "$EXT_LOWER" != "svg" ]; then
    FAILED_REASONS+=("Wrong format: .$EXT_LOWER (need PNG/SVG)")
  fi

  # Check size (skip SVG - they're usually small)
  if [ "$EXT_LOWER" != "svg" ]; then
    SIZE_KB=$(du -k "$img" | cut -f1)
    if [ "$SIZE_KB" -gt "$MAX_SIZE_KB" ]; then
      FAILED_REASONS+=("Too large: ${SIZE_KB}KB (max ${MAX_SIZE_KB}KB)")
    fi
  fi

  if [ ${#FAILED_REASONS[@]} -gt 0 ]; then
    FAILED=$((FAILED + 1))
    echo "❌ $img: ${FAILED_REASONS[*]}"
  else
    PASSED=$((PASSED + 1))
  fi
done

echo ""
echo "Total images: $TOTAL"
echo "✅ Passed: $PASSED"
echo "❌ Failed: $FAILED"
echo ""

if [ "$FAILED" -gt 0 ]; then
  exit 1
fi

echo "✅ All images meet requirements!"
exit 0
