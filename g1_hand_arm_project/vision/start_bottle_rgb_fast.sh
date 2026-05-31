#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="/home/unitree/unitree_sdk2_python/g1_hand_arm_project"
MODEL="/home/unitree/YOLO_Model_Workspace/Models/Training_Runs/bottle_v12/weights/best.pt"
SOURCE="/dev/video4"
LOG="/tmp/detect_bottle_2d.log"

pkill -f "vision/detect_bottle_2d.py" 2>/dev/null || true
sleep 1

cd "$PROJECT_DIR"
DISPLAY=:99 nohup python3 -u vision/detect_bottle_2d.py \
  --model "$MODEL" \
  --source "$SOURCE" \
  --width 640 \
  --height 480 \
  --show \
  --frames 0 \
  --imgsz 640 \
  --conf 0.15 \
  --infer-every 2 \
  --track-max-jump 80 \
  --track-smooth-alpha 0.35 \
  --track-lost-frames 12 \
  --track-switch-frames 8 \
  --track-lock-conf 0.25 \
  --flush-frames 0 \
  --save-every 0 \
  --print-every 30 \
  > "$LOG" 2>&1 &

echo "started RGB YOLO bottle detector"
echo "pid=$!"
echo "log=$LOG"
