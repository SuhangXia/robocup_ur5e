#!/bin/bash
# Download all required models and datasets for RoboCup UR5e system

set -e  # Exit on error

echo "╔══════════════════════════════════════════════════════════════╗"
echo "║         RoboCup UR5e - Model & Dataset Downloader           ║"
echo "╚══════════════════════════════════════════════════════════════╝"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

cd "$WORKSPACE_DIR"

# ============================================
# Configuration
# ============================================
YOLO_WEIGHTS_DIR="weights/yolo"
GRASPNET_WEIGHTS_DIR="weights/graspnet"
YCB_DATA_DIR="data/ycb_objects"

HUGGINGFACE_REPO="SuhangXia/robocup-ur5e-models"
USE_HUGGINGFACE=false

# ============================================
# Check Dependencies
# ============================================
echo "📋 Checking dependencies..."

if ! command -v wget &> /dev/null; then
    echo "❌ wget not found. Installing..."
    sudo apt-get update && sudo apt-get install -y wget
fi

if command -v huggingface-cli &> /dev/null; then
    USE_HUGGINGFACE=true
    echo "✅ Hugging Face CLI found"
else
    echo "⚠️  Hugging Face CLI not found (optional)"
    echo "   Install: pip install huggingface-hub"
    echo "   Will download from official sources instead"
fi

echo ""

# ============================================
# Function: Download from URL
# ============================================
download_file() {
    local url=$1
    local output=$2
    local description=$3
    
    echo "📥 Downloading $description..."
    if [ -f "$output" ]; then
        echo "   ✅ Already exists: $output"
        return 0
    fi
    
    echo "   URL: $url"
    wget -q --show-progress -O "$output" "$url"
    
    if [ $? -eq 0 ]; then
        echo "   ✅ Downloaded: $output"
    else
        echo "   ❌ Failed to download: $output"
        return 1
    fi
}

# ============================================
# Function: Download from Hugging Face
# ============================================
download_from_hf() {
    local repo=$1
    local file=$2
    local local_dir=$3
    local description=$4
    
    echo "📥 Downloading $description from Hugging Face..."
    
    if [ -f "$local_dir/$file" ]; then
        echo "   ✅ Already exists: $local_dir/$file"
        return 0
    fi
    
    huggingface-cli download "$repo" "$file" --local-dir "$local_dir"
    
    if [ $? -eq 0 ]; then
        echo "   ✅ Downloaded: $local_dir/$file"
    else
        echo "   ❌ Failed to download from Hugging Face"
        return 1
    fi
}

# ============================================
# 1. Download YOLO Weights
# ============================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🎯 Step 1: Download YOLO Weights"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

mkdir -p "$YOLO_WEIGHTS_DIR"

# YOLOv8 Nano (6MB) - Small enough to commit
YOLOV8N_URL="https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt"
download_file "$YOLOV8N_URL" "$YOLO_WEIGHTS_DIR/yolov8n.pt" "YOLOv8 Nano (6MB)"

echo ""
read -p "Download YOLOv8 Small (22MB)? Recommended for production. (yes/no): " download_yolov8s

if [ "$download_yolov8s" = "yes" ]; then
    if [ "$USE_HUGGINGFACE" = true ]; then
        download_from_hf "$HUGGINGFACE_REPO" "yolov8s.pt" "$YOLO_WEIGHTS_DIR" "YOLOv8 Small (22MB)"
    else
        YOLOV8S_URL="https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8s.pt"
        download_file "$YOLOV8S_URL" "$YOLO_WEIGHTS_DIR/yolov8s.pt" "YOLOv8 Small (22MB)"
    fi
fi

echo ""
echo "✅ YOLO weights download complete"
echo "   Location: $YOLO_WEIGHTS_DIR"
ls -lh "$YOLO_WEIGHTS_DIR"
echo ""

# ============================================
# 2. Download GraspNet Checkpoint
# ============================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🤏 Step 2: Download GraspNet Checkpoint"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

mkdir -p "$GRASPNET_WEIGHTS_DIR"

echo "⚠️  GraspNet checkpoint is ~600MB"
read -p "Download now? (yes/no): " download_graspnet

if [ "$download_graspnet" = "yes" ]; then
    if [ "$USE_HUGGINGFACE" = true ]; then
        echo "📥 Attempting Hugging Face download (faster)..."
        download_from_hf "$HUGGINGFACE_REPO" "graspnet_checkpoint.tar" "$GRASPNET_WEIGHTS_DIR" "GraspNet Checkpoint"
        
        if [ $? -ne 0 ]; then
            echo "⚠️  Hugging Face download failed, trying official source..."
            GRASPNET_URL="https://graspnet.net/models/checkpoint-rs.tar"
            download_file "$GRASPNET_URL" "$GRASPNET_WEIGHTS_DIR/checkpoint-rs.tar" "GraspNet Checkpoint (600MB)"
        fi
    else
        GRASPNET_URL="https://graspnet.net/models/checkpoint-rs.tar"
        download_file "$GRASPNET_URL" "$GRASPNET_WEIGHTS_DIR/checkpoint-rs.tar" "GraspNet Checkpoint (600MB)"
    fi
    
    # Extract if tar file exists
    if [ -f "$GRASPNET_WEIGHTS_DIR/checkpoint-rs.tar" ] || [ -f "$GRASPNET_WEIGHTS_DIR/graspnet_checkpoint.tar" ]; then
        echo "📦 Extracting checkpoint..."
        cd "$GRASPNET_WEIGHTS_DIR"
        tar -xf *.tar 2>/dev/null || true
        cd "$WORKSPACE_DIR"
        echo "✅ Extraction complete"
    fi
else
    echo "⏭️  Skipped GraspNet download"
    echo "   Note: System will use fallback grasp planner without GraspNet"
fi

echo ""
echo "Location: $GRASPNET_WEIGHTS_DIR"
ls -lh "$GRASPNET_WEIGHTS_DIR" 2>/dev/null || echo "   (empty)"
echo ""

# ============================================
# 3. Download YCB Objects (Optional)
# ============================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📊 Step 3: Download YCB Object Dataset (Optional)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

mkdir -p "$YCB_DATA_DIR"

echo "YCB dataset is used for Gazebo simulation and grasp planning"
echo "  - Essential objects: ~100MB (competition objects only)"
echo "  - Full dataset: ~2.5GB (all 77 objects)"
echo ""
read -p "Download YCB objects? (essential/full/no): " download_ycb

if [ "$download_ycb" = "essential" ]; then
    if [ "$USE_HUGGINGFACE" = true ]; then
        download_from_hf "$HUGGINGFACE_REPO" "ycb_essential.tar.gz" "$YCB_DATA_DIR" "YCB Essential Objects (100MB)"
        
        echo "📦 Extracting objects..."
        cd "$YCB_DATA_DIR"
        tar -xzf ycb_essential.tar.gz 2>/dev/null || true
        cd "$WORKSPACE_DIR"
    else
        echo "⚠️  Hugging Face CLI required for YCB download"
        echo "   Install: pip install huggingface-hub"
    fi
elif [ "$download_ycb" = "full" ]; then
    echo "📥 Downloading full YCB dataset (2.5GB)..."
    echo "   This may take 10-20 minutes depending on your connection"
    
    if command -v git-lfs &> /dev/null; then
        git lfs install
        git clone https://huggingface.co/datasets/SuhangXia/ycb-objects "$YCB_DATA_DIR/full"
    else
        echo "❌ Git LFS required for full dataset download"
        echo "   Install: sudo apt-get install git-lfs"
    fi
else
    echo "⏭️  Skipped YCB dataset download"
fi

echo ""

# ============================================
# Summary
# ============================================
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "✅ Download Complete!"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
echo "📊 Summary:"
echo ""

# Check YOLO
YOLO_COUNT=$(ls "$YOLO_WEIGHTS_DIR"/*.pt 2>/dev/null | wc -l)
echo "  🎯 YOLO: $YOLO_COUNT model(s)"
[ $YOLO_COUNT -gt 0 ] && ls -lh "$YOLO_WEIGHTS_DIR"/*.pt

# Check GraspNet
GRASPNET_COUNT=$(ls "$GRASPNET_WEIGHTS_DIR"/*.pth "$GRASPNET_WEIGHTS_DIR"/*.tar 2>/dev/null | wc -l)
echo "  🤏 GraspNet: $GRASPNET_COUNT file(s)"
[ $GRASPNET_COUNT -gt 0 ] && ls -lh "$GRASPNET_WEIGHTS_DIR" | head -5

# Check YCB
YCB_COUNT=$(find "$YCB_DATA_DIR" -name "*.obj" 2>/dev/null | wc -l)
echo "  📊 YCB Objects: $YCB_COUNT objects"

echo ""
echo "📂 File locations:"
echo "  - YOLO: $YOLO_WEIGHTS_DIR"
echo "  - GraspNet: $GRASPNET_WEIGHTS_DIR"
echo "  - YCB: $YCB_DATA_DIR"
echo ""
echo "📖 For more information, see: docs/MODELS_AND_DATASETS.md"
echo ""
echo "🚀 You can now start the system:"
echo "   ./scripts/start.sh"
