#!/usr/bin/env python3
"""Quick test to verify state monitoring works."""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'lib'))

from ros_utils import (
    get_autoware_state, get_route_state, get_mrm_state,
    shutdown_ros, AutowareState
)

print("Testing state monitor...")
print("=" * 50)

# Test autoware state
print("\n1. Testing Autoware state:")
state, name = get_autoware_state(timeout=5.0)
if state is not None:
    print(f"   SUCCESS: {name} (value: {state})")
else:
    print(f"   FAILED: Could not get state ({name})")

# Test route state
print("\n2. Testing Route state:")
state, name = get_route_state(timeout=5.0)
if state is not None:
    print(f"   SUCCESS: {name} (value: {state})")
else:
    print(f"   FAILED: Could not get state ({name})")

# Test MRM state
print("\n3. Testing MRM state:")
state, behavior = get_mrm_state(timeout=5.0)
if state is not None:
    print(f"   SUCCESS: state={state.name}, behavior={behavior.name if behavior else 'None'}")
else:
    print(f"   FAILED: Could not get MRM state")

# Continuous monitoring for 10 seconds
print("\n4. Continuous monitoring (10 seconds):")
start = time.time()
while time.time() - start < 10:
    aw_state, aw_name = get_autoware_state(timeout=1.0)
    route_state, route_name = get_route_state(timeout=1.0)
    mrm_state, mrm_behavior = get_mrm_state(timeout=1.0)
    
    mrm_str = mrm_state.name if mrm_state else "?"
    print(f"   [{time.time()-start:.1f}s] Autoware: {aw_name}, Route: {route_name}, MRM: {mrm_str}")
    time.sleep(1.0)

print("\n" + "=" * 50)
print("Test complete!")

shutdown_ros()
