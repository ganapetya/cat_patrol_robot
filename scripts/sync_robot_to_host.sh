#!/usr/bin/env bash
# =============================================================================
# sync_robot_to_host.sh
# =============================================================================
# Runs on the HOST machine (NOT on the Jetson).
#
# What it does, in order:
#   1. rsync the entire workspace from the Jetson to /home/bots/all/
#      (one-way mirror, the canonical "what's on the robot right now").
#   2. rsync cat_patrol_robot/   from the mirror to /opt/robots/cat_patrol_robot/
#   3. rsync odom_drift_checker/ from the mirror to /opt/robots/odom_drift_checker/
#   4. Copy the two modified Yahboom files into
#      /opt/robots/cat_patrol_robot/yahboom_overlay/  so they get tracked
#      with cat_patrol_robot's git repo (without polluting the upstream
#      Yahboom checkout on the robot).
#
# After it finishes, you do the normal git workflow in each /opt/robots
# project:
#     cd /opt/robots/cat_patrol_robot   && git add . && git commit && git push
#     cd /opt/robots/odom_drift_checker && git add . && git commit && git push
#
# This is a PUSH-ONLY workflow: the robot is the source of truth, and we
# never pull git changes back into the robot. Edits go robot → host → git.
# =============================================================================

set -uo pipefail
# NOTE: We deliberately do NOT use `set -e`. rsync legitimately returns
# nonzero on partial transfers (codes 23 / 24) when there are e.g. nested
# build dirs that don't sync cleanly. We want the script to keep going
# past those benign warnings and run steps 2 and 3 anyway.

ROBOT_HOST="${ROBOT_HOST:-jetson@192.168.0.120}"
ROBOT_WS="${ROBOT_WS:-/home/jetson/yahboomcar_ros2_ws}"
HOST_MIRROR="${HOST_MIRROR:-/home/bots/all}"
SHARED_BASE="${SHARED_BASE:-/opt/robots}"

# Packages to mirror to per-package shared dirs (one git repo each).
# Add new ones here as you create them.
TRACKED_PACKAGES=(
    cat_patrol_robot
    odom_drift_checker
)

# Vendor files to copy into cat_patrol_robot/yahboom_overlay/.
# Format: "src_relative_to_yahboomcar_ws/src    dst_relative_to_yahboom_overlay"
YAHBOOM_OVERLAY_FILES=(
    "yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py    yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py"
    "yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py    yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py"
)

echo "===================================================================="
echo "STEP 1/3: rsync from robot to host mirror"
echo "  ${ROBOT_HOST}:${ROBOT_WS}/  ->  ${HOST_MIRROR}/"
echo "===================================================================="
# Exclude colcon's build artefacts and ros logs -- they're not source-of-truth
# and the build/ dir occasionally contains symlink loops that confuse rsync.
rsync -avz --progress \
    --exclude='build/' --exclude='install/' --exclude='log/' \
    --exclude='__pycache__/' --exclude='*.pyc' \
    --exclude='.ros_inspect_logs/' \
    "${ROBOT_HOST}:${ROBOT_WS}/" "${HOST_MIRROR}/" \
    || echo "[warn] rsync from robot exited nonzero (likely benign partial-transfer); continuing"

WS_SRC="${HOST_MIRROR}/yahboomcar_ws/src"

echo
echo "===================================================================="
echo "STEP 2/3: rsync each tracked package to its shared dir"
echo "===================================================================="
for pkg in "${TRACKED_PACKAGES[@]}"; do
    src="${WS_SRC}/${pkg}/"
    dst="${SHARED_BASE}/${pkg}/"
    if [ ! -d "${src}" ]; then
        echo "  SKIP ${pkg}: ${src} not found in mirror"
        continue
    fi
    mkdir -p "${dst}"
    echo "  ${src}  ->  ${dst}"
    rsync -a --delete --exclude='.git' --exclude='build/' --exclude='install/' --exclude='log/' \
          --exclude='__pycache__/' --exclude='*.pyc' --exclude='.ros_inspect_logs/' \
          "${src}" "${dst}"
done

echo
echo "===================================================================="
echo "STEP 3/3: copy Yahboom-modified files into cat_patrol_robot/yahboom_overlay/"
echo "===================================================================="
OVERLAY_BASE="${SHARED_BASE}/cat_patrol_robot/yahboom_overlay"
for line in "${YAHBOOM_OVERLAY_FILES[@]}"; do
    # Split on whitespace.
    src_rel="$(echo "${line}" | awk '{print $1}')"
    dst_rel="$(echo "${line}" | awk '{print $2}')"
    src="${WS_SRC}/${src_rel}"
    dst="${OVERLAY_BASE}/${dst_rel}"
    if [ ! -f "${src}" ]; then
        echo "  SKIP: ${src} not found"
        continue
    fi
    mkdir -p "$(dirname "${dst}")"
    cp "${src}" "${dst}"
    echo "  ${src_rel}  ->  yahboom_overlay/${dst_rel}"
done

# Drop a tiny README into yahboom_overlay so future-you knows what it is.
cat > "${OVERLAY_BASE}/README.md" << 'EOF'
# yahboom_overlay

Files in this directory are **modifications to the Yahboom-shipped
`yahboomcar_bringup` package**. We keep them here (rather than committing
inside the Yahboom checkout) so a fresh clone of Yahboom's repo doesn't
clobber our changes.

To re-apply after re-cloning Yahboom:

    cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src
    cp -v cat_patrol_robot/yahboom_overlay/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py \
          yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py
    cp -v cat_patrol_robot/yahboom_overlay/yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py \
          yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py
    colcon build --symlink-install --packages-select yahboomcar_bringup

The diffs vs upstream are described in plan.md (Phase 0 findings) and
phase0-status.md.
EOF

echo
echo "===================================================================="
echo "Done. Next steps (manual):"
echo "===================================================================="
for pkg in "${TRACKED_PACKAGES[@]}"; do
    echo "  cd ${SHARED_BASE}/${pkg} && git status"
done
