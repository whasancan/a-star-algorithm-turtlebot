#!/bin/bash
# ===========================================
# Nav2 Goal Sender Script
# Kullanım: ./send_goal.sh x y
# Örnek:   ./send_goal.sh 1.5 0.8
# ===========================================

if [ "$#" -ne 2 ]; then
    echo "Kullanım: $0 <x> <y>"
    echo "Örnek:   $0 1.5 0.8"
    exit 1
fi

X=$1
Y=$2

echo "🎯 Goal gönderiliyor: x=$X, y=$Y"

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
