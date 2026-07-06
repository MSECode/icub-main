# legTrajectoryPlayer

`legTrajectoryPlayer` is a YARP `RFModule` that reads the MATLAB v7.3 walking
logger file directly as HDF5 and extracts only the iCub leg joint trajectories.

The default input paths are:

- positions: `/robot_logger_device/walking/joints_state/positions/desired`
- velocities: `/robot_logger_device/walking/joints_state/velocities/measured`

The extracted channels are:

- left leg: `l_hip_pitch`, `l_hip_roll`, `l_hip_yaw`, `l_knee`, `l_ankle_pitch`, `l_ankle_roll`
- right leg: `r_hip_pitch`, `r_hip_roll`, `r_hip_yaw`, `r_knee`, `r_ankle_pitch`, `r_ankle_roll`

The MAT data is assumed to be in radians/radians per second and is converted to
degrees/degrees per second before sending commands to YARP control boards.

## Examples

Dump the extracted leg data to CSV while also doing a dry run:

```sh
legTrajectoryPlayer \
  --mat /mnt/c/Users/jlosi/Workspace/study-encoders/MultiJoint_Setup_Analysis_Script/Data/walking-icub-gen11/walking-4-steps-starting-left.mat \
  --dryRun \
  --dump /tmp/walking-4-steps-legs.csv
```

Plot the dumped CSV before moving the robot:

```sh
python3 src/tools/legTrajectoryPlayer/helpers/plot_trajectory_csv.py \
  /tmp/walking-4-steps-legs.csv
```

Save a non-interactive plot for a quick report/check:

```sh
python3 src/tools/legTrajectoryPlayer/helpers/plot_trajectory_csv.py \
  /tmp/walking-4-steps-legs.csv \
  --legs left \
  --velocity-limit 80 \
  --output /tmp/walking-left-leg.png \
  --no-show
```

Move all six joints of both legs for five seconds with position-direct streaming:

```sh
legTrajectoryPlayer \
  --robot icub \
  --mat /mnt/c/Users/jlosi/Workspace/study-encoders/MultiJoint_Setup_Analysis_Script/Data/walking-icub-gen11/walking-4-steps-starting-left.mat \
  --duration 5.0 \
  --controlMode positionDirect
```

Move only the first three axes of the left leg:

```sh
legTrajectoryPlayer \
  --robot icub \
  --mat /path/to/walking-4-steps-starting-left.mat \
  --legs "(left)" \
  --joints "(0 1 2)" \
  --duration 5.0
```

Use `--leftJoints "(...)"` and `--rightJoints "(...)"` for different joint
subsets on the two legs. Local leg axis indices are:

`0 hip_pitch`, `1 hip_roll`, `2 hip_yaw`, `3 knee`, `4 ankle_pitch`,
`5 ankle_roll`.
