#!/bin/bash
# Script to create a clean, shareable version of YOLO-RescueSim
# Excludes build artifacts, virtual environments, and large binary files

set -e

PROJECT_DIR="/home/zubair/Downloads/YOLO-RescueSim"
ARCHIVE_NAME="YOLO-RescueSim-clean.tar.gz"
DEST_DIR="/home/zubair/Downloads"

echo "Creating clean archive of YOLO-RescueSim..."
echo "=========================================="

cd "$PROJECT_DIR"

# Create archive excluding heavy files
tar -czf "$DEST_DIR/$ARCHIVE_NAME" \
  --exclude='.venv' \
  --exclude='build' \
  --exclude='install' \
  --exclude='log' \
  --exclude='__pycache__' \
  --exclude='*.pyc' \
  --exclude='*.pyo' \
  --exclude='.git' \
  --exclude='*.bag' \
  --exclude='*.db3' \
  --exclude='*.mp4' \
  --exclude='*.avi' \
  .

ARCHIVE_SIZE=$(du -sh "$DEST_DIR/$ARCHIVE_NAME" | awk '{print $1}')

echo ""
echo "=========================================="
echo "✓ Archive created successfully!"
echo "Location: $DEST_DIR/$ARCHIVE_NAME"
echo "Size: $ARCHIVE_SIZE"
echo ""
echo "This archive contains:"
echo "  ✓ Source code (project/)"
echo "  ✓ Configuration files"
echo "  ✓ Launch files"
echo "  ✓ Documentation"
echo "  ✓ Requirements file"
echo "  ✓ Setup scripts"
echo "  ✓ YOLO model (yolov8n.pt)"
echo ""
echo "Excluded (can be regenerated):"
echo "  ✗ .venv/ (~7GB - Python packages)"
echo "  ✗ build/ (compiled artifacts)"
echo "  ✗ install/ (ROS install space)"
echo "  ✗ log/ (build logs)"
echo ""
echo "Recipients can recreate the environment by running:"
echo "  ./install_dependencies.sh"
echo "  ./complete_setup.sh"
echo "=========================================="
