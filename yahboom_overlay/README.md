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
