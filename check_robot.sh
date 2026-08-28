#!/usr/bin/env bash
# Run this WHILE the robot stack is up. Answers one question: is the arm actually able to
# execute commands, or is it only reporting its position?
source /opt/ros/jazzy/setup.bash
source /home/ros/share/mantis_ws/install/setup.bash

echo "=============== 1. КОНТРОЛЛЕРЫ ==============="
echo "forward_position_controller ОБЯЗАН быть active. inactive/unconfigured = команды в никуда."
timeout 20 ros2 control list_controllers 2>&1 | grep -vE "waiting|^\[INFO" || echo "  controller_manager НЕ ОТВЕЧАЕТ -> драйвер робота не запущен"

echo
echo "=============== 2. ПОТОК СОСТОЯНИЯ ==============="
timeout 8 ros2 topic hz /joint_states 2>/dev/null | grep -m1 "average rate" || echo "  /joint_states молчит"

echo
echo "=============== 3. КОМАНДНЫЙ ТОПИК ==============="
echo "Нужен хотя бы 1 подписчик — это контроллер. Ноль = команды никто не читает."
timeout 15 ros2 topic info /forward_position_controller/commands 2>/dev/null || echo "  топика нет"

echo
echo "=============== 4. РЕЖИМ И БЕЗОПАСНОСТЬ UR ==============="
timeout 15 ros2 topic list 2>/dev/null | grep -iE "robot_mode|safety_mode|robot_program_running|io_and_status" | while read -r t; do
  printf "  %-58s " "$t"
  timeout 5 ros2 topic echo --once "$t" 2>/dev/null | tr '\n' ' ' | head -c 90; echo
done
echo "  robot_mode: 7 = RUNNING (норма). Меньше -> робот не готов."
echo "  safety_mode: 1 = NORMAL. 3 = PROTECTIVE_STOP, 7 = EMERGENCY_STOP."
echo "  robot_program_running: true = программа на пульте идёт. false -> команды не исполняются."

echo
echo "=============== 5. ЗАХВАТ ==============="
timeout 12 ros2 action list 2>/dev/null | grep -i grip || echo "  action-сервера захвата НЕТ (в логе: gripper action server not ready)"
