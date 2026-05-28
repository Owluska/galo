#!/usr/bin/env python3
"""Staged parameter optimizer for the GALO ROS 2 pipeline.

The default search order is intentionally signal-first:
  1. planar feature extraction
  2. ground segmentation and patch quality
  3. wheel-model prediction
  4. gates and pose smoothing

Each stage runs coordinate search: try every value for one parameter while
holding the current best configuration fixed, keep improvements, then move to
the next parameter. This keeps the number of bag replays manageable while still
letting later stages inherit earlier wins.
"""

from __future__ import annotations

import argparse
import copy
import csv
import math
import os
import random
import re
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Iterable

import yaml


DEFAULT_CONFIG = Path(__file__).with_name("galo_param_sweep.yaml")


@dataclass(frozen=True)
class SearchParam:
    path: str
    values: tuple[Any, ...]


STAGES: dict[str, list[SearchParam]] = {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run staged GALO parameter optimization from one YAML config."
    )
    parser.add_argument(
        "config",
        nargs="?",
        default=str(DEFAULT_CONFIG),
        help="Path to sweep config YAML. Defaults to scripts/galo_param_sweep.yaml.",
    )
    return parser.parse_args()


def require_mapping(data: dict[str, Any], key: str) -> dict[str, Any]:
    value = data.get(key, {})
    if not isinstance(value, dict):
        raise SystemExit(f"Config key '{key}' must be a mapping")
    return value


def load_config(path: Path) -> argparse.Namespace:
    raw = yaml.safe_load(path.read_text())
    if not isinstance(raw, dict):
        raise SystemExit(f"Config file is empty or invalid: {path}")

    execution = require_mapping(raw, "execution")
    scoring = require_mapping(raw, "scoring")
    search = require_mapping(raw, "search")

    stages = search.get("stages", [])
    if not isinstance(stages, list) or not all(isinstance(stage, str) for stage in stages):
        raise SystemExit("Config key 'search.stages' must be a list of stage names")

    strategy = search.get("strategy", "coordinate")
    if strategy not in {"coordinate", "random"}:
        raise SystemExit("Config key 'search.strategy' must be 'coordinate' or 'random'")

    write_best = raw.get("write_best")
    if write_best is None:
        write_best = execution.get("write_best")

    return argparse.Namespace(
        config=str(path),
        bag=str(raw.get("bag")),
        base_params=str(raw.get("base_params")),
        out_dir=str(raw.get("out_dir")),
        setup=str(raw.get("setup")),
        stages=stages,
        strategy=strategy,
        random_trials=int(search.get("random_trials", 40)),
        random_seed=int(search.get("random_seed", 7)),
        passes=int(search.get("passes", 1)),
        rate=float(execution.get("rate", 1.0)),
        bag_timeout_sec=float(execution.get("bag_timeout_sec", 120.0)),
        startup_sec=float(execution.get("startup_sec", 3.0)),
        settle_sec=float(execution.get("settle_sec", 3.0)),
        min_rows=int(execution.get("min_rows", 5)),
        use_sim_time=bool(execution.get("use_sim_time", True)),
        launch_static_tf=bool(execution.get("launch_static_tf", True)),
        dry_run=bool(execution.get("dry_run", False)),
        resume=bool(execution.get("resume", False)),
        yaw_weight=float(scoring.get("yaw_weight", 2.0)),
        gate_penalty=float(scoring.get("gate_penalty", 2.0)),
        max_xy_penalty=float(scoring.get("max_xy_penalty", 0.05)),
        write_best=write_best,
    )


def load_sweep(path: Path) -> dict[str, list[SearchParam]]:
    raw = yaml.safe_load(path.read_text())
    sweep = raw.get("sweep") if isinstance(raw, dict) else None
    if not isinstance(sweep, dict) or not sweep:
        raise SystemExit("Config key 'sweep' must define stage parameter values")

    stages: dict[str, list[SearchParam]] = {}
    for stage_name, params in sweep.items():
        if not isinstance(params, dict):
            raise SystemExit(f"Config sweep stage '{stage_name}' must be a mapping")
        stage_params: list[SearchParam] = []
        for dotted_path, values in params.items():
            if not isinstance(values, list) or not values:
                raise SystemExit(
                    f"Config sweep value '{stage_name}.{dotted_path}' must be a non-empty list"
                )
            stage_params.append(SearchParam(str(dotted_path), tuple(values)))
        stages[str(stage_name)] = stage_params
    return stages


def clean_name(text: str) -> str:
    text = re.sub(r"[^A-Za-z0-9_.-]+", "_", text)
    return text.strip("_")[:120]


def get_path(data: dict[str, Any], dotted_path: str) -> Any:
    current: Any = data
    for part in dotted_path.split("."):
        current = current[part]
    return current


def set_path(data: dict[str, Any], dotted_path: str, value: Any) -> None:
    current: Any = data
    parts = dotted_path.split(".")
    for part in parts[:-1]:
        current = current[part]
    current[parts[-1]] = value


def load_base_params(path: Path) -> dict[str, Any]:
    raw = yaml.safe_load(path.read_text())
    return raw["galo"]["ros__parameters"]


def make_ros_params(base: dict[str, Any], csv_path: Path, use_sim_time: bool) -> dict[str, Any]:
    params = copy.deepcopy(base)
    params.setdefault("node", {})
    params["node"]["odom_error_csv_enabled"] = True
    params["node"]["odom_error_csv_path"] = str(csv_path)
    params["node"]["debug"] = 0
    params["use_sim_time"] = use_sim_time
    return {
        "galo_deskew": {"ros__parameters": copy.deepcopy(params)},
        "galo_frontend": {"ros__parameters": copy.deepcopy(params)},
        "galo": {"ros__parameters": copy.deepcopy(params)},
    }


def write_single_node_params(base: dict[str, Any], path: Path) -> None:
    path.write_text(
        yaml.safe_dump(
            {"galo": {"ros__parameters": copy.deepcopy(base)}},
            sort_keys=False,
        )
    )


def start_process(cmd: str, log_path: Path) -> tuple[subprocess.Popen[str], Any]:
    log = log_path.open("w")
    proc = subprocess.Popen(
        ["bash", "-lc", cmd],
        stdout=log,
        stderr=subprocess.STDOUT,
        preexec_fn=os.setsid,
        text=True,
    )
    return proc, log


def stop_process(proc: subprocess.Popen[str], timeout: float = 5.0) -> None:
    if proc.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            os.killpg(os.getpgid(proc.pid), sig)
            proc.wait(timeout=timeout)
            return
        except Exception:
            continue


def float_column(rows: list[dict[str, str]], name: str) -> list[float]:
    values = []
    for row in rows:
        try:
            value = float(row.get(name, "nan"))
        except ValueError:
            continue
        if math.isfinite(value):
            values.append(value)
    return values


def true_rate(rows: list[dict[str, str]], name: str) -> float:
    if not rows:
        return 0.0
    true_values = {"1", "1.0", "true", "True", "TRUE"}
    return sum(1 for row in rows if row.get(name) in true_values) / len(rows)


def read_metrics(csv_path: Path, args: argparse.Namespace) -> dict[str, Any]:
    if not csv_path.exists() or csv_path.stat().st_size == 0:
        return {"ok": False, "reason": "missing_csv", "score": math.inf}

    with csv_path.open(newline="") as handle:
        rows = list(csv.DictReader(handle))
    if len(rows) < args.min_rows:
        return {
            "ok": False,
            "reason": f"too_few_rows:{len(rows)}",
            "rows": len(rows),
            "score": math.inf,
        }

    err_x = float_column(rows, "error_x")
    err_y = float_column(rows, "error_y")
    err_z = float_column(rows, "error_z")
    err_roll = float_column(rows, "error_roll")
    err_pitch = float_column(rows, "error_pitch")
    err_yaw = float_column(rows, "error_yaw")

    xy_pairs = list(zip(err_x, err_y))
    xy_rmse = math.sqrt(sum(x * x + y * y for x, y in xy_pairs) / max(1, len(xy_pairs)))
    z_rmse = math.sqrt(sum(z * z for z in err_z) / max(1, len(err_z)))
    rp_rmse = math.sqrt(
        sum(r * r + p * p for r, p in zip(err_roll, err_pitch))
        / max(1, min(len(err_roll), len(err_pitch)))
    )
    yaw_rmse = math.sqrt(sum(y * y for y in err_yaw) / max(1, len(err_yaw)))
    max_xy = max((math.hypot(x, y) for x, y in xy_pairs), default=math.inf)
    final_xy = math.hypot(xy_pairs[-1][0], xy_pairs[-1][1]) if xy_pairs else math.inf
    planar_ok_rate = true_rate(rows, "planar_gate_ok")
    mean_planar_matches = sum(float_column(rows, "planar_matches")) / max(
        1, len(float_column(rows, "planar_matches"))
    )
    mean_planar_residual = sum(float_column(rows, "planar_mean_residual")) / max(
        1, len(float_column(rows, "planar_mean_residual"))
    )

    score = (
        xy_rmse
        + args.yaw_weight * yaw_rmse
        + 0.2 * z_rmse
        + 0.2 * rp_rmse
        + args.max_xy_penalty * max_xy
        + 0.1 * final_xy
        + args.gate_penalty * max(0.0, 0.75 - planar_ok_rate)
    )

    return {
        "ok": True,
        "reason": "",
        "rows": len(rows),
        "score": score,
        "xy_rmse": xy_rmse,
        "z_rmse": z_rmse,
        "rp_rmse": rp_rmse,
        "yaw_rmse": yaw_rmse,
        "max_xy": max_xy,
        "final_xy": final_xy,
        "planar_ok_rate": planar_ok_rate,
        "mean_planar_matches": mean_planar_matches,
        "mean_planar_residual": mean_planar_residual,
    }


def command_with_setup(setup: str, command: str) -> str:
    return f"source {setup} && exec {command}"


def run_candidate(
    name: str,
    params: dict[str, Any],
    args: argparse.Namespace,
    out_dir: Path,
) -> dict[str, Any]:
    csv_path = out_dir / f"{name}.csv"
    params_path = out_dir / f"{name}.yaml"
    log_prefix = out_dir / name

    if args.resume and csv_path.exists() and csv_path.stat().st_size > 0:
        metrics = read_metrics(csv_path, args)
        return {**metrics, "name": name, "csv": str(csv_path), "params_file": str(params_path), "resumed": True}

    for path in (csv_path, params_path):
        if path.exists():
            path.unlink()

    params_path.write_text(
        yaml.safe_dump(make_ros_params(params, csv_path, args.use_sim_time), sort_keys=False)
    )

    if args.dry_run:
        return {
            "ok": True,
            "reason": "dry_run",
            "score": math.inf,
            "name": name,
            "csv": str(csv_path),
            "params_file": str(params_path),
        }

    processes: list[tuple[subprocess.Popen[str], Any]] = []
    bag_proc: subprocess.Popen[str] | None = None
    bag_log = None
    try:
        if args.launch_static_tf:
            cmd = command_with_setup(
                args.setup,
                "ros2 launch static_tf_publisher static_tf.launch.py",
            )
            processes.append(start_process(cmd, log_prefix.with_suffix(".static_tf.log")))

        node_commands = [
            f"ros2 run ground_aware_lidar_odometry deskew_node --ros-args --params-file {params_path}",
            f"ros2 run ground_aware_lidar_odometry frontend_node --ros-args --params-file {params_path}",
            f"ros2 run ground_aware_lidar_odometry node --ros-args --params-file {params_path}",
        ]
        for index, node_command in enumerate(node_commands):
            processes.append(
                start_process(
                    command_with_setup(args.setup, node_command),
                    log_prefix.with_suffix(f".node{index}.log"),
                )
            )

        time.sleep(args.startup_sec)
        bag_log = log_prefix.with_suffix(".bag.log").open("w")
        bag_command = command_with_setup(
            args.setup,
            f"ros2 bag play {args.bag} --clock 100 --rate {args.rate}",
        )
        bag_proc = subprocess.Popen(
            ["bash", "-lc", bag_command],
            stdout=bag_log,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
            text=True,
        )
        bag_proc.wait(timeout=args.bag_timeout_sec)
        time.sleep(args.settle_sec)
    finally:
        if bag_proc is not None:
            stop_process(bag_proc)
        if bag_log is not None:
            bag_log.close()
        for proc, log in processes:
            stop_process(proc)
            log.close()

    metrics = read_metrics(csv_path, args)
    return {
        **metrics,
        "name": name,
        "csv": str(csv_path),
        "params_file": str(params_path),
        "resumed": False,
    }


def summarize_result(result: dict[str, Any]) -> str:
    if not result.get("ok"):
        return f"{result['name']}: FAILED {result.get('reason', '')}"
    if result.get("reason") == "dry_run":
        return f"{result['name']}: wrote {result['params_file']}"
    return (
        f"{result['name']}: score={result['score']:.4f} "
        f"xy={result.get('xy_rmse', math.nan):.3f} "
        f"yaw={result.get('yaw_rmse', math.nan):.4f} "
        f"max_xy={result.get('max_xy', math.nan):.3f} "
        f"planar_ok={result.get('planar_ok_rate', math.nan):.2f} "
        f"matches={result.get('mean_planar_matches', math.nan):.1f}"
    )


def write_summary(results: list[dict[str, Any]], out_dir: Path) -> None:
    if not results:
        return
    keys = sorted({key for result in results for key in result})
    with (out_dir / "summary.csv").open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=keys)
        writer.writeheader()
        writer.writerows(results)


def coordinate_candidates(
    current: dict[str, Any],
    stage: str,
    pass_index: int,
) -> Iterable[tuple[str, dict[str, Any], str, Any]]:
    for param in STAGES[stage]:
        current_value = get_path(current, param.path)
        for value in param.values:
            if value == current_value:
                continue
            candidate = copy.deepcopy(current)
            set_path(candidate, param.path, value)
            suffix = clean_name(f"p{pass_index}_{param.path}_{value}")
            yield suffix, candidate, param.path, value


def random_candidates(
    current: dict[str, Any],
    stage: str,
    trials: int,
    rng: random.Random,
) -> Iterable[tuple[str, dict[str, Any], str, str]]:
    for trial in range(trials):
        candidate = copy.deepcopy(current)
        changed = []
        for param in STAGES[stage]:
            value = rng.choice(param.values)
            set_path(candidate, param.path, value)
            changed.append(f"{param.path}={value}")
        yield f"random_{trial:03d}", candidate, "random", ";".join(changed)


def run_stage(
    stage: str,
    best_params: dict[str, Any],
    best_result: dict[str, Any],
    args: argparse.Namespace,
    out_dir: Path,
    results: list[dict[str, Any]],
    rng: random.Random,
) -> tuple[dict[str, Any], dict[str, Any]]:
    print(f"\n=== Stage: {stage} ===", flush=True)

    if args.strategy == "coordinate":
        for pass_index in range(1, args.passes + 1):
            candidate_index = 0
            for param in STAGES[stage]:
                for value in param.values:
                    if value == get_path(best_params, param.path):
                        continue
                    candidate_index += 1
                    params = copy.deepcopy(best_params)
                    set_path(params, param.path, value)
                    suffix = clean_name(f"p{pass_index}_{param.path}_{value}")
                    name = clean_name(f"{stage}_{suffix}")
                    print(
                        f"[{stage} pass {pass_index} #{candidate_index}] "
                        f"{param.path} -> {value}",
                        flush=True,
                    )
                    result = run_candidate(name, params, args, out_dir)
                    result["stage"] = stage
                    result["changed_path"] = param.path
                    result["changed_value"] = value
                    results.append(result)
                    print("  " + summarize_result(result), flush=True)
                    write_summary(results, out_dir)

                    if result.get("ok") and result["score"] < best_result["score"]:
                        print(
                            f"  new best: {result['score']:.4f} < "
                            f"{best_result['score']:.4f}",
                            flush=True,
                        )
                        best_params = copy.deepcopy(params)
                        best_result = result
    else:
        candidates = random_candidates(best_params, stage, args.random_trials, rng)
        for candidate_index, (suffix, params, changed_path, changed_value) in enumerate(candidates, 1):
            name = clean_name(f"{stage}_{suffix}")
            print(
                f"[{stage} pass 1 #{candidate_index}] {changed_path} -> {changed_value}",
                flush=True,
            )
            result = run_candidate(name, params, args, out_dir)
            result["stage"] = stage
            result["changed_path"] = changed_path
            result["changed_value"] = changed_value
            results.append(result)
            print("  " + summarize_result(result), flush=True)
            write_summary(results, out_dir)

            if result.get("ok") and result["score"] < best_result["score"]:
                print(
                    f"  new best: {result['score']:.4f} < {best_result['score']:.4f}",
                    flush=True,
                )
                best_params = copy.deepcopy(params)
                best_result = result

    return best_params, best_result


def validate_stages(stage_names: Iterable[str]) -> list[str]:
    stages = [stage.strip() for stage in stage_names if stage.strip()]
    unknown = [stage for stage in stages if stage not in STAGES]
    if unknown:
        raise SystemExit(f"Unknown stages: {unknown}. Choices: {sorted(STAGES)}")
    return stages


def main() -> int:
    cli_args = parse_args()
    config_path = Path(cli_args.config)
    args = load_config(config_path)
    global STAGES
    STAGES = load_sweep(config_path)
    stages = validate_stages(args.stages)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    base_params = load_base_params(Path(args.base_params))
    rng = random.Random(args.random_seed)
    results: list[dict[str, Any]] = []

    baseline_name = "baseline"
    print(f"Using sweep config {config_path}", flush=True)
    print(f"Writing outputs to {out_dir}", flush=True)
    print(f"Running baseline from {args.base_params}", flush=True)
    baseline_result = run_candidate(baseline_name, base_params, args, out_dir)
    baseline_result["stage"] = "baseline"
    baseline_result["changed_path"] = ""
    baseline_result["changed_value"] = ""
    results.append(baseline_result)
    print("  " + summarize_result(baseline_result), flush=True)

    best_params = copy.deepcopy(base_params)
    best_result = baseline_result
    if args.dry_run:
        best_result = {**baseline_result, "score": math.inf}
    elif not baseline_result.get("ok"):
        print("Baseline failed; continuing, but no candidate can be compared until one succeeds.", flush=True)
        best_result = {**baseline_result, "score": math.inf}

    for stage in stages:
        best_params, best_result = run_stage(
            stage, best_params, best_result, args, out_dir, results, rng
        )

    write_summary(results, out_dir)
    best_path = Path(args.write_best) if args.write_best else out_dir / "best_galo_params.yaml"
    write_single_node_params(best_params, best_path)

    print("\n=== Best ===", flush=True)
    print(summarize_result(best_result), flush=True)
    print(f"Summary: {out_dir / 'summary.csv'}", flush=True)
    print(f"Best params: {best_path}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
