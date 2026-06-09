# Collision tuning note

This package version reduces over-conservative collision boxes that were limiting MoveIt planning for `left_arm` and `right_arm`.

Modified files:

- `zhishu_robot_description/urdf/zhishu_robot_description_dh.urdf`
  - Reduced base/torso collision boxes.
  - Reduced left/right arm link collision boxes.
  - Reduced hand collision boxes.
- `zhishu_robot_description/urdf/gripper.xacro`
  - Reduced gripper collision boxes only; visual geometry unchanged.
- `zhishu_robot_moveit_config/config/zhishu_robot.srdf`
  - Added disabled collision between `R_hand_link` and `gripper_right_finger_link`.

After rebuilding, open RViz collision display and verify that the current state is not red. If collision pairs still appear near the shoulders or hands, regenerate the Self-Collision Matrix in MoveIt Setup Assistant.
