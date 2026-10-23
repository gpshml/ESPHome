#!/bin/bash
# Cabin Bed Project - File Verification Script

echo "════════════════════════════════════════════════════"
echo "  CABIN BED PROJECT - FILE VERIFICATION"
echo "════════════════════════════════════════════════════"
echo ""

# Color codes
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

missing=0
found=0

# Check for required files
echo "Checking technical drawings..."
files_drawings=("front_elevation.png" "top_plan.png" "side_elevation.png" "3d_visualization.png" "drawer_detail.png" "drawer_side_detail.png")

for file in "${files_drawings[@]}"; do
    if [ -f "$file" ]; then
        echo -e "  ${GREEN}✓${NC} $file ($(du -h "$file" | cut -f1))"
        ((found++))
    else
        echo -e "  ${RED}✗${NC} $file - MISSING!"
        ((missing++))
    fi
done

echo ""
echo "Checking documentation..."
files_docs=("cabin-bed-project.html" "cabin-bed-specifications.md" "UPDATED-SPECIFICATIONS.md" "materials_calculation.txt" "README.md" "QUICK-REFERENCE.md" "PROJECT-SUMMARY.md" "START-HERE.md")

for file in "${files_docs[@]}"; do
    if [ -f "$file" ]; then
        echo -e "  ${GREEN}✓${NC} $file ($(du -h "$file" | cut -f1))"
        ((found++))
    else
        echo -e "  ${RED}✗${NC} $file - MISSING!"
        ((missing++))
    fi
done

echo ""
echo "Checking source code..."
if [ -f "generate_cabin_bed_drawings.py" ]; then
    echo -e "  ${GREEN}✓${NC} generate_cabin_bed_drawings.py ($(du -h "generate_cabin_bed_drawings.py" | cut -f1))"
    ((found++))
else
    echo -e "  ${RED}✗${NC} generate_cabin_bed_drawings.py - MISSING!"
    ((missing++))
fi

echo ""
echo "════════════════════════════════════════════════════"
echo -e "  Found: ${GREEN}${found}${NC} files"
if [ $missing -gt 0 ]; then
    echo -e "  Missing: ${RED}${missing}${NC} files"
else
    echo -e "  Missing: ${GREEN}0${NC} files"
fi
echo "════════════════════════════════════════════════════"

if [ $missing -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ ALL FILES PRESENT - READY TO BUILD!${NC}"
    echo ""
    echo "Next steps:"
    echo "  1. Read START-HERE.md"
    echo "  2. Read UPDATED-SPECIFICATIONS.md"
    echo "  3. Open cabin-bed-project.html in browser"
    echo ""
else
    echo ""
    echo -e "${RED}⚠ Some files are missing!${NC}"
    echo "Please ensure all files are downloaded to the same folder."
    echo ""
fi

echo "════════════════════════════════════════════════════"
