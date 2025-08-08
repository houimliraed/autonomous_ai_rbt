#!/bin/bash

# GitHub Upload Preparation Script for AutonomBot
# This script prepares the project for upload to your other GitHub account

echo "=========================================="
echo "🚀 AUTONOMBOT GITHUB PREPARATION SCRIPT"
echo "=========================================="

# Color codes
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}📝 Preparing project for GitHub upload...${NC}"

# Clean up large files and build artifacts
echo -e "${YELLOW}🧹 Cleaning build artifacts and large files...${NC}"

# Remove build directories
rm -rf build/ install/ log/

# Remove large model files (they'll be downloaded automatically)
find . -name "*.pt" -not -name "yolov8n.pt" -delete

# Remove performance logs
rm -f system_performance_report_*.json

# Remove RViz temp files  
rm -f .rviz_*

echo -e "${GREEN}✓ Cleaned build artifacts${NC}"

# Check for secrets or sensitive information
echo -e "${YELLOW}🔍 Checking for sensitive information...${NC}"

# List of patterns to check for
sensitive_patterns=(
    "password"
    "secret"
    "api_key" 
    "token"
    "private_key"
    "ssh_key"
)

for pattern in "${sensitive_patterns[@]}"; do
    if grep -r -i "$pattern" src/ 2>/dev/null; then
        echo -e "${YELLOW}⚠️  Found potential sensitive data: $pattern${NC}"
        echo "Please review and remove before uploading to GitHub"
    fi
done

echo -e "${GREEN}✓ Security check completed${NC}"

# Update README with your GitHub username
echo -e "${YELLOW}📝 Please update the following before pushing to GitHub:${NC}"
echo ""
echo "1. Update README.md badges with your GitHub username:"
echo "   - Replace 'YOUR_USERNAME' with your actual GitHub username"
echo ""
echo "2. Update CI/CD workflow file:"
echo "   - Update .github/workflows/ci.yml with your repository details"
echo ""
echo "3. Update Docker files if needed:"
echo "   - Update maintainer email in docker/Dockerfile"

# Initialize git repository if not already done
if [ ! -d ".git" ]; then
    echo -e "${YELLOW}🔧 Initializing Git repository...${NC}"
    git init
    git add .
    git commit -m "Initial commit: AutonomBot autonomous navigation robot"
    echo -e "${GREEN}✓ Git repository initialized${NC}"
else
    echo -e "${YELLOW}📦 Git repository already exists${NC}"
    echo "Current status:"
    git status --short
fi

# Show file sizes
echo -e "${YELLOW}📊 Repository size analysis:${NC}"
du -sh . 
echo ""
echo "Largest files:"
find . -type f -size +1M -exec du -sh {} \; 2>/dev/null | sort -hr | head -5

echo ""
echo -e "${BLUE}🚀 GITHUB UPLOAD INSTRUCTIONS:${NC}"
echo "============================================="
echo ""
echo "1. Create new repository on your other GitHub account:"
echo "   - Repository name: autonombot"
echo "   - Description: Autonomous navigation robot with YOLO vision"
echo "   - Make it public for better visibility"
echo ""
echo "2. Update repository URLs in this project:"
echo "   - README.md: Replace YOUR_USERNAME with your GitHub username"
echo "   - CI/CD files: Update repository references"
echo ""
echo "3. Push to GitHub:"
echo "   git remote add origin https://github.com/YOUR_USERNAME/autonombot.git"
echo "   git branch -M main"
echo "   git push -u origin main"
echo ""
echo "4. Enable GitHub Actions:"
echo "   - Go to repository Settings > Actions"
echo "   - Enable 'Allow all actions and reusable workflows'"
echo ""
echo "5. Set up branch protection (recommended):"
echo "   - Go to Settings > Branches"
echo "   - Add protection rule for 'main' branch"
echo "   - Require status checks to pass"
echo "   - Require pull request reviews"
echo ""
echo -e "${GREEN}✅ Project is ready for GitHub upload!${NC}"
echo ""
echo "Repository structure:"
tree -I 'build|install|log|__pycache__' -L 2
