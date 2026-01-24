#!/bin/bash
# Script to commit and push documentation updates to GitHub

cd "$(dirname "$0")/../../.." || exit 1

echo "=========================================="
echo "Updating GitHub with documentation changes"
echo "=========================================="
echo ""

# Check if we're in a git repository
if ! git rev-parse --git-dir > /dev/null 2>&1; then
    echo "Error: Not a git repository"
    exit 1
fi

# Show current status
echo "Current git status:"
git status --short
echo ""

# Add the documentation files
echo "Adding documentation files..."
git add ros_package/aae4011_ai_uas/WSL2_SETUP.md
git add ros_package/aae4011_ai_uas/README.md

# Check if there are changes to commit
if git diff --staged --quiet; then
    echo "No changes to commit."
    exit 0
fi

# Commit the changes
echo "Committing changes..."
git commit -m "Update WSL2 setup documentation and add rosbag information

- Revised WSL2_SETUP.md to reflect new workflow: clone from GitHub directly in WSL2
- Updated setup instructions for using Cursor IDE in WSL2
- Added rosbag download information (campus_small_dataset.bag) with Google Drive link
- Updated README.md with rosbag download instructions
- Added multiple download methods for rosbag file"

# Push to GitHub
echo ""
echo "Pushing to GitHub..."
if git push origin main; then
    echo ""
    echo "=========================================="
    echo "✓ Successfully pushed to GitHub!"
    echo "=========================================="
else
    echo ""
    echo "=========================================="
    echo "✗ Failed to push to GitHub"
    echo "You may need to run: git push origin main"
    echo "=========================================="
    exit 1
fi
