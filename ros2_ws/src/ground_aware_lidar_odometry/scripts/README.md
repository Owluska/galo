# GALO Parameter Sweep

`galo_param_sweep.py` replays a ROS 2 bag repeatedly while testing staged
parameter changes for the GALO pipeline. The whole experiment is configured in
`galo_param_sweep.yaml`: bag path, base params, output directory, scoring, search
strategy, stage order, and every sweep value.

## Config-First Usage

Run with the packaged config:

```bash
python3 ground_aware_lidar_odometry/scripts/galo_param_sweep.py
```

Run with a custom config:

```bash
python3 ground_aware_lidar_odometry/scripts/galo_param_sweep.py \
  /tmp/my_galo_sweep.yaml
```

That config path is the only script argument. For a new experiment, duplicate
`galo_param_sweep.yaml`, edit it, and run the script with the new path.

## Config Layout

Top-level runtime paths:

```yaml
bag: /ros2_ws/BAGS/109_2025-08-14-19-24-29_1212e_ros2
base_params: /ros2_ws/src/ground_aware_lidar_odometry/config/galo_params.yaml
out_dir: /tmp/galo_param_sweep
setup: /ros2_ws/install/setup.bash
write_best:
```

Execution settings:

```yaml
execution:
  use_sim_time: true
  launch_static_tf: true
  dry_run: false
  resume: false
  rate: 1.0
  bag_timeout_sec: 120.0
  startup_sec: 3.0
  settle_sec: 3.0
  min_rows: 5
```

Search settings:

```yaml
search:
  strategy: coordinate
  passes: 1
  random_trials: 40
  random_seed: 7
  stages:
    - planar
    - ground
    - prediction
    - gates_smoothing
```

Sweep values live under `sweep`. Each stage maps a GALO dotted parameter path to
a list of values:

```yaml
sweep:
  planar:
    planar_registration.voxel_size: [2.0, 3.0, 5.0, 7.0]
    planar_registration.min_points_per_voxel: [4, 8, 10, 15]
```

## Search Order

The default order tunes the signal source before the trust layer:

1. `planar` - planar line extraction and planar registration feature settings.
2. `ground` - ground segmentation and ground patch quality.
3. `prediction` - wheel-model prediction parameters.
4. `gates_smoothing` - registration gates and final pose blending.

`coordinate` search tries one parameter value at a time, keeps improvements, and
carries the best result into the next parameter and stage. `random` samples full
combinations from all parameter lists in a stage.

## Common Experiments

Dry-run only writes candidate YAMLs and does not start ROS processes:

```yaml
execution:
  dry_run: true
```

Run only planar and ground tuning:

```yaml
search:
  stages: [planar, ground]
```

Run a smaller randomized pass:

```yaml
search:
  strategy: random
  random_trials: 30
```

Resume from candidate CSVs already present in `out_dir`:

```yaml
execution:
  resume: true
```

## Outputs

The output directory contains:

- `summary.csv` - one row per candidate, including score and diagnostics.
- `best_galo_params.yaml` - best parameter set in normal GALO YAML form.
- `<candidate>.yaml` - ROS params used for each candidate run.
- `<candidate>.csv` - odometry error CSV produced by the GALO node.
- `<candidate>.*.log` - logs from static TF, GALO nodes, and bag replay.

Lower `score` is better. The score combines xy RMSE, yaw RMSE, z and roll/pitch
error, max/final xy error, and a penalty for low planar gate rate.

## Recommended Flow

Start with `search.stages: [planar, ground]`. Once feature quality looks stable,
run `prediction`, then finish with `gates_smoothing`. Keep each experiment config
next to its output summary so the winner is reproducible.
