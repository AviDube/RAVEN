#!/usr/bin/env bash
# Fully tear down and recreate the RAVEN stack (Isaac Sim, robot/AirStack, and
# RayFronts) from scratch every time.
#
# Why this exists: soft-restarting individual pieces (e.g. just re-launching a
# process inside an already-running container) leaves stale ROS2/DDS
# participants and connections behind -- this shows up as "sequence size
# exceeds remaining buffer" spam, frozen RayFronts frame processing, or
# parameter-service timeouts. Fully recreating all three containers guarantees
# every ROS2 node starts with a clean DDS session.
#
# Usage: ./restart_raven_fresh.sh [query_file]
#   query_file - path *inside* the rayfront_docker container to the RayFronts
#                query file (default: construction site vocab)

set -euo pipefail

AIRSTACK_DIR="/home/avid/RAVEN/AirStack"
RAYFRONTS_COMPOSE_DIR="/home/avid/docker"
QUERY_FILE="${1:-/RayFronts/junbin_site_queries.json}"
MAPPING_LOG="/RayFronts/mapping_server_construction.log"

RAYFRONTS_CMD="source /opt/ros/humble/setup.bash && cd /RayFronts && HYDRA_FULL_ERROR=1 python3 -m rayfronts.mapping_server_rosnode dataset=ros2isaacsim mapping=semantic_ray_frontiers_map mapping.vox_size=0.5 dataset.rgb_resolution=[448,448] dataset.depth_resolution=[448,448] dataset.frame_skip=10 mapping.max_rays_per_frame=50 mapping.vox_accum_period=2 mapping.occ_observ_weight=100 mapping.max_occ_cnt=100 mapping.ray_accum_period=4 mapping.ray_accum_phase=2 querying.query_file=${QUERY_FILE}"

log() { echo -e "\n==> $*"; }

log "Recreating isaac-sim + robot containers from scratch..."
cd "$AIRSTACK_DIR"
set -a
source .env
set +a
docker compose -f docker-compose.yaml up -d --force-recreate isaac-sim robot

log "Waiting for PX4 SITL to come up inside isaac-sim (up to 120s)..."
px4_ok=false
for _ in $(seq 1 24); do
  if docker exec isaac-sim bash -c "pgrep -f bin/px4" >/dev/null 2>&1; then
    px4_ok=true
    break
  fi
  sleep 5
done
if [ "$px4_ok" = true ]; then
  echo "PX4 process detected."
else
  echo "WARNING: PX4 process not detected after 120s -- check 'docker exec isaac-sim bash -c \"tmux capture-pane -t isaac -p -S -100\"'"
fi

log "Recreating rayfront_docker container from scratch (model cache is persisted on host, so this stays fast)..."
cd "$RAYFRONTS_COMPOSE_DIR"
docker compose up -d --force-recreate

log "Waiting for rayfront_docker to be reachable..."
until docker exec rayfront_docker true 2>/dev/null; do sleep 1; done

log "Letting ROS2/DDS discovery settle across the fresh containers (30s)..."
sleep 30

log "Launching RayFronts mapping server fresh (query file: ${QUERY_FILE})..."
docker exec rayfront_docker bash -c "rm -f ${MAPPING_LOG}" || true
docker exec -d rayfront_docker bash -c "${RAYFRONTS_CMD} > ${MAPPING_LOG} 2>&1"

# Best-effort: if a `raven` tmux session from a prior run is still around,
# repoint its rayfronts/input windows at the fresh container so `tmux attach
# -t raven` shows live output instead of a dead pane. Never fatal.
if tmux has-session -t raven 2>/dev/null; then
  log "Repointing existing 'raven' tmux session windows at the fresh containers..."
  tmux send-keys -t raven:rayfronts "clear && docker exec -it rayfront_docker bash -c 'tail -f -n 100 ${MAPPING_LOG}'" ENTER 2>/dev/null || true
  tmux send-keys -t raven:input C-c 2>/dev/null || true
  sleep 1
  tmux send-keys -t raven:input "clear && docker exec -it rayfront_docker bash -ic 'cd /RayFronts && python3 input_prompt.py'" ENTER 2>/dev/null || true
fi

log "Done. Fresh stack is up."
echo "    Isaac Sim / robot: docker exec -it isaac-sim bash -c 'tmux attach -t isaac'"
echo "    RayFronts log:     docker exec -it rayfront_docker bash -c 'tail -f ${MAPPING_LOG}'"
echo "    Give a target:     docker exec -it rayfront_docker bash -ic 'cd /RayFronts && python3 input_prompt.py'"
echo
echo "Remaining manual GUI steps (Isaac Sim auto-plays via PLAY_SIM_ON_START):"
echo "    1. Click Arm and Takeoff in the RQT-GUI"
echo "    2. Click Global Plan in the RQT-GUI"
