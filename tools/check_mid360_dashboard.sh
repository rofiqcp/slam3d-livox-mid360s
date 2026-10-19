#!/usr/bin/env bash
# Non-invasive smoke test: HTTP, ROS topics and service availability (no motor, no map writes).
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd -P)"
source /opt/ros/humble/setup.bash
source "$ROOT/env.sh"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "== MID360 dashboard HTTP =="
curl --fail --silent --show-error --max-time 4 http://127.0.0.1:5001/api/health
echo
python3 - <<'PY'
import urllib.request,json
root='http://127.0.0.1:5001'
for suffix in ['/api/status','/api/points']:
    with urllib.request.urlopen(root+suffix,timeout=5) as r:
        data=json.load(r)
        assert r.status==200
        if suffix.endswith('status'):
            assert isinstance(data['sensor_online'],bool)
            assert isinstance(data['map_available'],bool)
            print('sensor_online:',data['sensor_online'],'imu_online:',data['imu_online'],
                  'map_available:',data['map_available'],'map_points:',data['map_points'])
        else:
            assert isinstance(data['points'],list)
            assert all(len(p)==4 for p in data['points'])
            print('preview points:',len(data['points']))
with urllib.request.urlopen(root+'/',timeout=5) as r:
    text=r.read().decode()
    assert r.status==200 and '<canvas' in text and 'MID-360S' in text
    print('HTML + 3D canvas: OK')
PY
echo "== ROS2 mapping services / topics =="
ros2 service list | grep -E '/p4/save_map|/mid360/save_preview' || true
ros2 topic list | grep -E '/mid360/map_preview|/p4/optimized_map|/livox/lidar' || true
echo "PASS: HTTP contract; sensor/map health is reported separately."
