#!/usr/bin/env bash
# 비상용 전체 종료. 평소엔 ros2 launch 의 Ctrl+C 로 충분 (PX4/gz 까지 같이 정리됨).
# 런치가 죽어서 뭔가 남았을 때만 쓴다.
pkill -9 -f px4; pkill -9 -f 'gz sim'; pkill -9 -f ruby
pkill -9 -f gzserver; pkill -9 -f gzclient
pkill -9 -f parameter_bridge; pkill -9 -f arms_
pkill -9 -f sitl_bridge
pkill -9 -f balloon_referee
sleep 3
ros2 daemon stop; sleep 1; ros2 daemon start
echo "✅ 전체 종료 완료"
