#!/bin/bash
# Prepare GitHub repository for submission
# This script helps you set up and push your code to GitHub

set -e

echo "=========================================="
echo "GitHub Repository Preparation"
echo "=========================================="
echo ""

# Check if git is installed
if ! command -v git &> /dev/null; then
    echo "ERROR: git is not installed!"
    echo ""
    echo "Install git:"
    echo "  Ubuntu/Debian: sudo apt-get install git"
    echo "  macOS: brew install git"
    echo "  Windows: Download from https://git-scm.com/"
    echo ""
    exit 1
fi

# Get current directory
REPO_DIR=$(pwd)
echo "Repository directory: $REPO_DIR"
echo ""

# Check if already a git repository
if [ -d ".git" ]; then
    echo "This is already a git repository."
    echo ""
    read -p "Do you want to reinitialize? (y/n): " REINIT
    if [ "$REINIT" = "y" ]; then
        echo "Backing up .git directory..."
        mv .git .git.backup.$(date +%Y%m%d_%H%M%S)
        git init
    fi
else
    echo "Initializing git repository..."
    git init
fi

echo ""
echo "=========================================="
echo "Step 1: Configure Git"
echo "=========================================="
echo ""

# Check git config
GIT_NAME=$(git config user.name || echo "")
GIT_EMAIL=$(git config user.email || echo "")

if [ -z "$GIT_NAME" ]; then
    read -p "Enter your name: " GIT_NAME
    git config user.name "$GIT_NAME"
fi

if [ -z "$GIT_EMAIL" ]; then
    read -p "Enter your email: " GIT_EMAIL
    git config user.email "$GIT_EMAIL"
fi

echo "Git configured:"
echo "  Name:  $GIT_NAME"
echo "  Email: $GIT_EMAIL"
echo ""

echo "=========================================="
echo "Step 2: Review Files to Commit"
echo "=========================================="
echo ""

# Show what will be added
echo "Files to be committed:"
echo ""
git add -n .
echo ""

read -p "Proceed with adding these files? (y/n): " PROCEED
if [ "$PROCEED" != "y" ]; then
    echo "Aborted."
    exit 0
fi

echo ""
echo "Adding files to git..."
git add .

echo ""
echo "=========================================="
echo "Step 3: Create Commit"
echo "=========================================="
echo ""

# Check if there are changes to commit
if git diff --cached --quiet; then
    echo "No changes to commit."
else
    echo "Creating commit..."
    git commit -m "Complete UAV Target Detection and Engagement System

- ROS Noetic + Gazebo simulation
- YOLOv5 object detection
- DeepSORT tracking with re-identification
- Autonomous flight control
- Visual servoing
- Target engagement system
- Comprehensive documentation
- Docker support for Windows
- Automated testing and analysis tools"

    echo ""
    echo "Commit created successfully!"
fi

echo ""
echo "=========================================="
echo "Step 4: GitHub Repository Setup"
echo "=========================================="
echo ""

echo "Now you need to create a repository on GitHub:"
echo ""
echo "1. Go to https://github.com/new"
echo "2. Repository name: uav-target-detection-simulation"
echo "3. Description: Autonomous UAV system for target detection, tracking, and engagement using ROS and Gazebo"
echo "4. Visibility: Public"
echo "5. Do NOT initialize with README (we have one)"
echo "6. Click 'Create repository'"
echo ""

read -p "Have you created the GitHub repository? (y/n): " CREATED
if [ "$CREATED" != "y" ]; then
    echo ""
    echo "Please create the repository on GitHub first, then run this script again."
    exit 0
fi

echo ""
read -p "Enter your GitHub username: " GITHUB_USER
read -p "Enter repository name (default: uav-target-detection-simulation): " REPO_NAME
REPO_NAME=${REPO_NAME:-uav-target-detection-simulation}

GITHUB_URL="https://github.com/$GITHUB_USER/$REPO_NAME.git"

echo ""
echo "GitHub repository URL: $GITHUB_URL"
echo ""

# Check if remote already exists
if git remote | grep -q "origin"; then
    echo "Remote 'origin' already exists. Updating URL..."
    git remote set-url origin "$GITHUB_URL"
else
    echo "Adding remote 'origin'..."
    git remote add origin "$GITHUB_URL"
fi

echo ""
echo "=========================================="
echo "Step 5: Push to GitHub"
echo "=========================================="
echo ""

echo "Pushing to GitHub..."
echo ""

# Try to push
if git push -u origin main 2>/dev/null; then
    echo ""
    echo "✓ Successfully pushed to GitHub!"
elif git push -u origin master 2>/dev/null; then
    echo ""
    echo "✓ Successfully pushed to GitHub!"
else
    echo ""
    echo "Push failed. This might be because:"
    echo "1. The branch name is different (try 'master' instead of 'main')"
    echo "2. Authentication failed (you may need to use a personal access token)"
    echo "3. The repository already has content"
    echo ""
    echo "Try manually:"
    echo "  git branch -M main"
    echo "  git push -u origin main"
    echo ""
    echo "Or if using master branch:"
    echo "  git push -u origin master"
    echo ""
    exit 1
fi

echo ""
echo "=========================================="
echo "Success!"
echo "=========================================="
echo ""
echo "Your repository is now on GitHub:"
echo "  $GITHUB_URL"
echo ""
echo "View it at:"
echo "  https://github.com/$GITHUB_USER/$REPO_NAME"
echo ""
echo "Next steps:"
echo "  1. Visit the repository URL to verify"
echo "  2. Check that all files are present"
echo "  3. Verify README.md displays correctly"
echo "  4. Copy the repository URL for submission"
echo ""
echo "Repository URL for submission:"
echo "  https://github.com/$GITHUB_USER/$REPO_NAME"
echo ""
echo "=========================================="
echo ""

# Save URL to file for easy reference
echo "https://github.com/$GITHUB_USER/$REPO_NAME" > GITHUB_REPOSITORY_URL.txt
echo "Repository URL saved to: GITHUB_REPOSITORY_URL.txt"
echo ""

echo "Done!"

