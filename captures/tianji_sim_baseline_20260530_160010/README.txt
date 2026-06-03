Tianji SIM baseline capture (reference for real robot)
====================================================

This bundle is the gold standard: read_only + TJ SDK IK + mapped_tf.
Do NOT change tianji_arm_node logic when comparing; fix real-robot config/path only.

Files:
  main.log              - full ros2 launch stdout/stderr
  config_snapshot/      - yaml + static TF + git state at capture time
  baseline.bag/         - rosbag2: /tf, joint_command/state, ee_pose, zsp_para
  topics/*.csv          - parallel CSV echoes (joint + EE pose)
  observers/
    debug_arm_axis.log  - chest->arm / chest->tianji_right (3s)
    tf_sample.log       - periodic tf2_echo snapshots
    ros2_graph.txt      - node/topic list after teleop up

Real robot later should match:
  - Same controller_config (tianji_output_sim.yaml) except read_only:false
  - Same static_transforms.yaml
  - Compare joint_command vs this bag's /tianji_arm/right/joint_command
  - Compare [READ_ONLY_POSE] right_target_xyzabc vs real Set B cmd / FK

Motion script: motion_script.txt (in this bundle)
Motion log:    motion_log.txt   (stamp steps from host: scripts/tianji_motion_stamp.sh)
Analyze log:   ./scripts/analyze_tianji_baseline_log.sh $MAIN_LOG
