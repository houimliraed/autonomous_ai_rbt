#!/bin/bash

# GitHub Repository Setup Script for AutonomBot
# Customize this script with your GitHub account details

set -e

echo "=========================================="
echo "🚀 AUTONOMBOT GITHUB SETUP"
echo "=========================================="

# CUSTOMIZE THESE VARIABLES WITH YOUR GITHUB INFO
GITHUB_USERNAME="houimliraed"                    # Your GitHub username
GITHUB_EMAIL="houimliraed@outlook.fr"            # Your GitHub email
REPO_NAME="autonomous_ai_rbt"                    # Your repository name
REPO_DESCRIPTION="Autonomous Navigation Robot with YOLO Vision and Multi-Sensor Fusion"

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m'

log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if variables are customized
check_customization() {
    if [ "$GITHUB_USERNAME" = "YOUR_GITHUB_USERNAME" ]; then
        log_error "Please customize GITHUB_USERNAME in this script first!"
        log_info "Edit the script and replace 'YOUR_GITHUB_USERNAME' with your actual GitHub username"
        exit 1
    fi
    
    if [ "$GITHUB_EMAIL" = "your.email@example.com" ]; then
        log_error "Please customize GITHUB_EMAIL in this script first!"
        log_info "Edit the script and replace 'your.email@example.com' with your actual GitHub email"
        exit 1
    fi
}

# Initialize git repository
init_git() {
    log_info "Initializing Git repository..."
    
    if [ ! -d ".git" ]; then
        git init
    fi
    
    # Configure git user
    git config user.name "$GITHUB_USERNAME"
    git config user.email "$GITHUB_EMAIL"
    
    log_info "Git repository initialized and configured"
}

# Clean up files that shouldn't be in git
cleanup_files() {
    log_info "Cleaning up temporary files..."
    
    # Remove build artifacts
    rm -rf build/ install/ log/
    
    # Remove temporary files
    find . -name "*.pyc" -delete
    find . -name "*.pyo" -delete
    find . -name "__pycache__" -type d -exec rm -rf {} + 2>/dev/null || true
    find . -name "*.log" -delete
    
    # Remove system performance reports (too large)
    rm -f system_performance_report_*.json
    
    log_info "Cleanup completed"
}

# Update README with GitHub info
update_readme() {
    log_info "Updating README with GitHub information..."
    
    # Update badge URLs
    sed -i "s/YOUR_USERNAME/$GITHUB_USERNAME/g" README.md
    
    # Update clone URL
    sed -i "s|git clone https://github.com/YOUR_USERNAME/autonombot.git|git clone https://github.com/$GITHUB_USERNAME/$REPO_NAME.git|g" README.md
    
    log_info "README updated with your GitHub details"
}

# Add all files to git
add_files() {
    log_info "Adding files to Git..."
    
    git add .
    git add .github/
    git add .gitignore
    
    log_info "Files added to Git staging"
}

# Create initial commit
create_commit() {
    log_info "Creating initial commit..."
    
    git commit -m "🚀 Initial commit: Complete AutonomBot project

✨ Features:
- Autonomous navigation with Nav2 stack
- YOLOv8 computer vision system
- Multi-sensor fusion (LiDAR, camera, ultrasonic)
- Jetson Xavier deployment ready
- Comprehensive CI/CD pipeline
- Real-time obstacle detection and avoidance

🛠️ Hardware Support:
- NVIDIA Jetson Xavier AGX
- RPLiDAR A1/A2
- Intel RealSense D435i
- HC-SR04 ultrasonic sensors
- Differential drive system

📦 Package Structure:
- autonombot_launch: System startup and configuration
- autonombot_drivers: Motor control and kinematics
- autonombot_description: URDF models and robot description
- autonombot_firmware: Hardware interfaces and drivers
- autonombot_localization: EKF and AMCL localization
- autonombot_mapping: SLAM and map generation
- autonombot_navigation: Path planning and navigation
- autonombot_vision: YOLO object detection

🎯 Ready for production deployment!"
    
    log_info "Initial commit created"
}

# Instructions for GitHub
show_github_instructions() {
    echo ""
    echo -e "${BLUE}=========================================="
    echo "📋 NEXT STEPS FOR GITHUB UPLOAD"
    echo -e "==========================================${NC}"
    echo ""
    echo "1. Create repository on GitHub:"
    echo "   - Go to https://github.com/new"
    echo "   - Repository name: $REPO_NAME"
    echo "   - Description: $REPO_DESCRIPTION"
    echo "   - Make it Public (recommended for open source)"
    echo "   - Don't initialize with README, .gitignore, or license (we have them)"
    echo ""
    echo "2. Add remote and push:"
    echo "   git remote add origin https://github.com/$GITHUB_USERNAME/$REPO_NAME.git"
    echo "   git branch -M main"
    echo "   git push -u origin main"
    echo ""
    echo "3. Configure repository settings:"
    echo "   - Enable GitHub Actions (for CI/CD)"
    echo "   - Add repository topics: ros2, autonomous-robot, computer-vision, jetson"
    echo "   - Set up branch protection rules for main branch"
    echo ""
    echo "4. Optional enhancements:"
    echo "   - Add repository description and website link"
    echo "   - Create GitHub Pages for documentation"
    echo "   - Set up issue templates"
    echo "   - Configure GitHub Discussions"
    echo ""
    echo -e "${GREEN}✅ Repository is ready for GitHub upload!${NC}"
}

# Generate project statistics
show_project_stats() {
    echo ""
    echo -e "${BLUE}📊 PROJECT STATISTICS${NC}"
    echo "=========================================="
    
    # Count files by type
    echo "📁 Files:"
    echo "   Python files: $(find . -name "*.py" | wc -l)"
    echo "   Launch files: $(find . -name "*.launch.py" | wc -l)"
    echo "   YAML configs: $(find . -name "*.yaml" | wc -l)"
    echo "   URDF files: $(find . -name "*.urdf.xacro" | wc -l)"
    echo "   Total files: $(find . -type f | wc -l)"
    
    # Count lines of code
    echo ""
    echo "📝 Lines of code:"
    echo "   Python: $(find . -name "*.py" -exec wc -l {} + 2>/dev/null | tail -1 | awk '{print $1}' || echo '0')"
    echo "   XML/Launch: $(find . -name "*.xml" -o -name "*.launch.py" -exec wc -l {} + 2>/dev/null | tail -1 | awk '{print $1}' || echo '0')"
    
    # Package count
    echo ""
    echo "📦 Packages: $(find src -maxdepth 1 -type d | grep -v "^src$" | wc -l)"
    
    echo "=========================================="
}

# Main execution
main() {
    log_info "Starting GitHub repository preparation..."
    
    check_customization
    cleanup_files
    init_git
    update_readme
    add_files
    create_commit
    show_project_stats
    show_github_instructions
    
    echo ""
    echo -e "${GREEN}🎉 SUCCESS! Repository is ready for GitHub upload!${NC}"
    echo -e "${YELLOW}📝 Don't forget to customize this script with your GitHub details before running!${NC}"
}

# Run main function
main "$@"
