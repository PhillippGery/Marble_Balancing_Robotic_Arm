#!/usr/bin/env python3
"""
verify_tensorboard_logging.py
─────────────────────────────
Diagnostic script to check TensorBoard logging without stopping training.

Run this in a SEPARATE terminal while training is running:
  python3 verify_tensorboard_logging.py

Does NOT require Gazebo or any ROS2 processes to be running.
"""

import os
import sys
import glob
from pathlib import Path
from datetime import datetime, timedelta


def check_tfevents_files(tb_dir):
    """Check for TensorFlow events files in the TensorBoard directory."""
    print("\n" + "=" * 80)
    print("1. CHECKING FOR .tfevents FILES")
    print("=" * 80)

    if not os.path.exists(tb_dir):
        print(f"✗ TensorBoard directory DOES NOT EXIST: {tb_dir}")
        print(f"  → Expected location: {os.path.abspath(tb_dir)}")
        return False

    print(f"✓ TensorBoard directory exists: {os.path.abspath(tb_dir)}")

    # Find all .tfevents files
    tfevents = glob.glob(os.path.join(tb_dir, '**', 'events.out.tfevents.*'), recursive=True)

    if not tfevents:
        print("\n✗ NO .tfevents FILES FOUND!")
        print("  → This means TensorBoard hasn't written any data yet")
        print("  → This is expected if training just started (<10 seconds)")

        # Check if directory is empty
        contents = os.listdir(tb_dir)
        if contents:
            print(f"\n  But directory has contents: {contents}")
        else:
            print("\n  Directory is completely empty (may indicate logger not initialized)")
        return False

    print(f"\n✓ Found {len(tfevents)} event file(s):")
    for i, f in enumerate(tfevents, 1):
        size_mb = os.path.getsize(f) / (1024 * 1024)
        mod_time = os.path.getmtime(f)
        mod_dt = datetime.fromtimestamp(mod_time)
        age_sec = (datetime.now() - mod_dt).total_seconds()
        age_str = f"{age_sec:.0f} seconds ago" if age_sec < 60 else f"{age_sec/60:.0f} minutes ago"

        print(f"  [{i}] {os.path.basename(f)}")
        print(f"      Size: {size_mb:.1f} MB")
        print(f"      Last modified: {age_str}")

    # Check if files are being updated
    print("\n  Checking if files are being actively written to...")
    oldest_mtime = min(os.path.getmtime(f) for f in tfevents)
    newest_mtime = max(os.path.getmtime(f) for f in tfevents)
    age_diff = (datetime.fromtimestamp(newest_mtime) - datetime.fromtimestamp(oldest_mtime)).total_seconds()

    if age_diff < 5:
        print("  ✓ Files being actively written (updated within last 5 seconds)")
    elif age_diff < 60:
        print(f"  ⚠ Files updated {age_diff:.0f}s ago (slow update)")
    else:
        print(f"  ✗ Files NOT being updated recently ({age_diff/60:.0f} minutes old)")

    return True


def check_tensorboard_structure(tb_dir):
    """Analyze structure of TensorBoard event files."""
    print("\n" + "=" * 80)
    print("2. CHECKING EVENT FILE STRUCTURE")
    print("=" * 80)

    try:
        from tensorboard.compat.proto import event_pb2
        from tensorflow.compat.v1 import EventAccumulator
    except ImportError:
        print("⚠ TensorFlow not available, skipping event file analysis")
        print("  Install with: pip install tensorboard tensorflow")
        return

    tfevents = glob.glob(os.path.join(tb_dir, '**', 'events.out.tfevents.*'), recursive=True)
    if not tfevents:
        print("✗ No event files to analyze")
        return

    event_file = tfevents[0]
    print(f"Analyzing: {os.path.basename(event_file)}")

    try:
        # Use EventAccumulator to read the file
        ea = EventAccumulator(os.path.dirname(event_file))
        ea.Reload()

        print(f"\n✓ Event file is valid TensorFlow format")

        # List all scalars logged
        scalars = ea.Tags()['scalars']
        print(f"\n✓ Found {len(scalars)} scalar metric(s) in event file:")
        for scalar in sorted(scalars):
            print(f"  • {scalar}")

        # Show recent values for first few scalars
        if scalars:
            print(f"\nRecent values for first 3 metrics:")
            for scalar in sorted(scalars)[:3]:
                try:
                    events = ea.Scalars(scalar)
                    if events:
                        latest = events[-1]
                        print(f"  {scalar}: {latest.value:.6f} (step {latest.step})")
                except Exception as e:
                    print(f"  {scalar}: <error reading> {e}")

    except Exception as e:
        print(f"✗ Error reading event file: {e}")
        print("  This may indicate corruption or file still being written")


def check_run_directories(tb_dir):
    """Check for run subdirectories (TB groups runs by subdirs)."""
    print("\n" + "=" * 80)
    print("3. CHECKING RUN DIRECTORY STRUCTURE")
    print("=" * 80)

    if not os.path.exists(tb_dir):
        print(f"✗ TensorBoard directory doesn't exist: {tb_dir}")
        return

    subdirs = [d for d in os.listdir(tb_dir) if os.path.isdir(os.path.join(tb_dir, d))]

    if subdirs:
        print(f"✓ Found {len(subdirs)} run subdirectory(ies):")
        for subdir in subdirs:
            subpath = os.path.join(tb_dir, subdir)
            tfevents = glob.glob(os.path.join(subpath, 'events.out.tfevents.*'))
            print(f"  • {subdir}/ — {len(tfevents)} event files")
    else:
        print("ℹ No subdirectories (event files directly in TB directory)")

    # Check for TD3_1, TD3_2 etc (indicates multiple runs)
    td3_dirs = [d for d in subdirs if d.startswith('TD3')]
    if len(td3_dirs) > 1:
        print(f"\n⚠ Found {len(td3_dirs)} TD3 run directories:")
        for td3_dir in sorted(td3_dirs):
            print(f"  • {td3_dir}")
        print("  → This may indicate tb_log_name not being consistent across runs")
        print("     (TensorBoard creates separate plots for each)")


def check_training_log():
    """Check if training script is outputting debug info."""
    print("\n" + "=" * 80)
    print("4. TRAINING SCRIPT CONFIGURATION")
    print("=" * 80)

    script_path = os.path.join(
        os.path.dirname(__file__), '..', 'src', 'marble_balancer', 'rl_training', 'train_td3_gazebo_v2.py'
    )

    if not os.path.exists(script_path):
        print(f"⚠ Could not find training script at {script_path}")
        return

    with open(script_path, 'r') as f:
        content = f.read()

    # Check for key configurations
    checks = {
        'Monitor wrapper': 'from stable_baselines3.common.monitor import Monitor',
        'env = Monitor(': 'env = Monitor(',
        '_setup_learn()': '_setup_learn(',
        'tb_log_name=': 'tb_log_name=',
        'model.learn()': 'model.learn(',
        'model.train()': 'model.train(',
    }

    print("Checking training script configuration:")
    for check_name, pattern in checks.items():
        found = pattern in content
        status = "✓" if found else "✗"
        print(f"  {status} {check_name}: {pattern}")

    # Specific warnings
    if 'from stable_baselines3.common.monitor import Monitor' not in content:
        print("\n⚠ WARNING: Monitor wrapper NOT imported")
        print("  → Episode-level metrics (reward, length) won't be logged")

    if 'model.learn(' in content and 'model.train(' in content and content.count('model.train(') > 2:
        print("\n⚠ WARNING: Using model.train() in manual loop")
        print("  → Callbacks may not be invoked automatically")


def main():
    """Main diagnostic routine."""
    print("\n")
    print("╔" + "=" * 78 + "╗")
    print("║" + " " * 78 + "║")
    print("║  TensorBoard Logging Verification Script (train_td3_gazebo_v2.py)".ljust(79) + "║")
    print("║" + " " * 78 + "║")
    print("╚" + "=" * 78 + "╝")

    # Determine TensorBoard directory
    script_dir = os.path.dirname(os.path.abspath(__file__))
    
    # Try multiple possible locations
    possible_dirs = [
        os.path.join(script_dir, 'src', 'marble_balancer', 'rl_training', 'tensorboard_td3_v2'),
        os.path.join(script_dir, 'tensorboard_td3_v2'),
        'tensorboard_td3_v2',
        os.path.expanduser('~/marble_balancing_logs/tensorboard_td3_v2'),
    ]

    tb_dir = None
    for candidate in possible_dirs:
        if os.path.exists(candidate):
            tb_dir = candidate
            break

    if not tb_dir:
        print("\nSearching for tensorboard_td3_v2 directory...")
        tb_dir = 'tensorboard_td3_v2'  # Default
        print(f"Using default: {tb_dir}")
        print(f"Absolute path: {os.path.abspath(tb_dir)}")

    # Run checks
    tfevents_ok = check_tfevents_files(tb_dir)
    check_tensorboard_structure(tb_dir)
    check_run_directories(tb_dir)
    check_training_log()

    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)

    if tfevents_ok:
        print("✓ TensorBoard logging appears to be working")
        print("\nTo view in browser:")
        print(f"  tensorboard --logdir {os.path.abspath(tb_dir)}")
        print("  → Open browser to http://localhost:6006")
    else:
        print("✗ TensorBoard logging may not be working")
        print("\nPossible issues:")
        print("  1. Training just started (takes ~10 seconds to first log)")
        print("  2. Monitor wrapper not added to environment")
        print("  3. Logger not initialized before training")
        print("  4. Working directory changed (relative path issues)")
        print("\nRefer to TENSORBOARD_DIAGNOSTIC.md for fixes")

    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
