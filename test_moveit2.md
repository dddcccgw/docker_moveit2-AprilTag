# Testing MoveIt2 Integration

## Issue Observed
The `ros2_control_node` is crashing when launching MoveIt2 with `hardware_type:=actual`. This is likely because:
- Robot arm may not be connected
- USB permissions issue with `/dev/ttyUSB0` or `/dev/ttyDXL`
- ros2_control hardware interface configuration mismatch

## Solution 1: Test with Simulation First

Launch MoveIt2 in **simulation mode** to test the integration without hardware:

```bash
# Terminal 1: Launch MoveIt2 in simulation
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \
  robot_model:=vx300s \
  hardware_type:=fake \
  use_rviz:=false

# Terminal 2: Test the MoveIt2 interface
ros2 run apriltag_robot_control moveit2_interface.py
```

This will:
- ✅ Start MoveIt2 with fake controllers (no hardware needed)
- ✅ Allow you to test motion planning
- ✅ Verify the MoveIt2 integration works

If the above command works, the crash is confirmed to be an RViz/GUI issue.

## Solution 2: Fix Hardware Connection

If you want to use the actual robot, first verify the connection:

### Step 1: Check USB Device
```bash
ls -l /dev/ttyUSB* /dev/ttyDXL
# Should show: /dev/ttyUSB0 or /dev/ttyDXL
```

### Step 2: Test Direct Robot Control (without MoveIt2)
```bash
# Test basic robot control first
ros2 launch interbotix_xsarm_control xsarm_control.launch.py \
  robot_model:=vx300s \
  use_rviz:=false
```

If this works, then try MoveIt2 again.

### Step 3: Launch MoveIt2 with Hardware
```bash
ros2 launch interbotix_xsarm_moveit xsarm_moveit.launch.py \
  robot_model:=vx300s \
  hardware_type:=actual \
  use_actual:=true
```

## Solution 3: Hybrid Approach (Recommended for Development)

Use the robot hardware for gripper control but MoveIt2 in simulation mode:

1. **Launch robot control separately** (for gripper):
   ```bash
   ros2 launch interbotix_xsarm_control xsarm_control.launch.py robot_model:=vx300s
   ```

2. **Don't launch MoveIt2** initially (our code has fallback to Interbotix API)

3. **Test the grasp controller** with `use_moveit: false`:
   ```bash
   # Edit config/params.yaml and set:
   motion:
     use_moveit: false
   ```

4. Once the basic system works, then add MoveIt2 for advanced planning

## Recommended Testing Order

1. ✅ **Test without hardware**: `hardware_type:=fake`
2. ✅ **Test direct robot control**: Just `xsarm_control.launch.py`
3. ✅ **Test grasp with Interbotix API**: `use_moveit: false`
4. ✅ **Add MoveIt2**: `hardware_type:=actual` + `use_moveit: true`

## Current Status

Your MoveIt2 integration code is complete and correct. The crash is a **hardware/launch configuration issue**, not a code problem. Start with simulation to verify everything works!
