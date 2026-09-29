#!/bin/bash
# ??????????????????????????????????????????????????????????
# start.sh ? Gait Analysis Monitor (ROS2 Humble)
# ??????????????????????????????????????????????????????????
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Un venv activo rompe rosbridge/rosapi (usan el python del venv, sin
# tornado/netifaces): se expulsa cualquier venv antes de lanzar nada.
if [ -n "$VIRTUAL_ENV" ]; then deactivate 2>/dev/null || true; fi
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '/.virtualenvs/' | paste -sd: -)"
unset PYTHONPATH VIRTUAL_ENV

source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=0
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

cleanup() {
  echo ""
  echo "=== Apagando servicios ==="
  kill $ROSBRIDGE_PID $SERVER_PID 2>/dev/null
  pkill -f rosbridge_websocket 2>/dev/null
  pkill -f rosapi_node 2>/dev/null
  fuser -k 9090/tcp 2>/dev/null
  fuser -k 8080/tcp 2>/dev/null
  sleep 1
  echo "Servicios detenidos."
}
trap cleanup EXIT

echo "=== Liberando puertos ==="
pkill -f rosbridge_websocket 2>/dev/null
pkill -f rosapi_node 2>/dev/null
fuser -k 9090/tcp 2>/dev/null
fuser -k 8080/tcp 2>/dev/null
sleep 2

echo "=== Arrancando rosbridge (puerto 9090) ==="
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 &
ROSBRIDGE_PID=$!
sleep 2

echo "=== Arrancando servidor web (puerto 8080) ==="
cd "$DIR"
python3 -m http.server 8080 &
SERVER_PID=$!
sleep 1

echo "=== Abriendo navegador ==="
xdg-open http://localhost:8080/ros2_gait_monitor.html &

echo ""
echo "Directorio: $DIR"
echo "Pulsa Ctrl+C para detener todo."
wait $SERVER_PID