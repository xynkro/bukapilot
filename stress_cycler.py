#!/usr/bin/env python3
import os, sys, time, signal, subprocess
from datetime import datetime, timezone, timedelta
from pathlib import Path

MAX_CYCLES, ONROAD, OFFROAD = 2, 35, 5
BASEDIR = Path(__file__).resolve().parent
test_proc = None

# Force UTC+8 absolute tracking timezone layout
TZ_SG_MY = timezone(timedelta(hours=8))

# DISCONNECT_TIMEOUT from hardwared.py: wait before going offroad after disconnect
# This ensures openpilot's hardwared completes the offroad transition
HARWARE_DISCONNECT_TIMEOUT = 6  # slightly more than hardwared's 5s DISCONNECT_TIMEOUT

def get_timestamp():
  return datetime.now(TZ_SG_MY).strftime('%X')

def run_cmd(cmd, shell=False):
  subprocess.run(cmd, shell=shell, check=False, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def stop_harness():
  """Destroys process groups instantly to ensure no half-dead processes corrupt memory rings."""
  global test_proc
  if test_proc:
    try: os.killpg(os.getpgid(test_proc.pid), signal.SIGKILL)
    except: pass
    test_proc = None
  for p in ("test_ka2_onroad.py", "ka2_can_replay_feeder.py"): run_cmd(["pkill", "-9", "-f", p])
  try:
    while (pid_status := os.waitpid(-1, os.WNOHANG))[0] > 0: pass
  except: pass

def set_gov(gov):
  if os.path.exists(node := "/sys/devices/system/cpu/cpufreq/policy0/scaling_governor"):
    run_cmd(f"echo {gov} | sudo tee {node}", shell=True)

def cleanup():
  stop_harness()
  for p in ("launch_openpilot.sh", "manager.py", "system.manager.manager", "./manager", "cereal"): run_cmd(["pkill", "-9", "-f", p])
  run_cmd(["sync"])
  set_gov("schedutil")

def sleep_track(secs, desc):
  end, last_printed = time.monotonic() + secs, -1
  while time.monotonic() < end:
    if (rem := max(0, int(round(end - time.monotonic())))) != last_printed:
      print(f"\r⏳ [{get_timestamp()}] {desc}: {rem}s remaining... ", end="", flush=True)
      last_printed = rem
    time.sleep(0.1)
  print("\r", end="", flush=True)

def wait_for_offroad():
  """
  Wait for openpilot's hardwared to complete the offroad transition.
  
  openpilot goes offroad when pandaStates stop being published:
  1. hardwared waits DISCONNECT_TIMEOUT (5s) after last pandaState
  2. Then sets ignition=False, which eventually sets started=False
  3. This triggers offroad transition in manager
  
  We wait HARWARE_DISCONNECT_TIMEOUT after killing the feeder to ensure
  hardwared has completed the transition.
  """
  sleep_track(HARWARE_DISCONNECT_TIMEOUT, "Waiting for offroad transition")

def handler(signum, frame):
  print(f"\n🛑 [{get_timestamp()}] User interrupt captured via Ctrl+C. Cleaning env context...")
  cleanup()
  sys.exit(0)

signal.signal(signal.SIGINT, handler)
cleanup()

os.environ["PYTHONUNBUFFERED"] = "1"

for cycle in range(1, MAX_CYCLES + 1):
  print(f"\n[{get_timestamp()}] 🔄 EXECUTING TEST CYCLE {cycle} OF {MAX_CYCLES} 🔄")

  # --- PHASE 1: HARDWARE PREPARATION & COLD BUFFER BOOT ---
  set_gov("performance")
  time.sleep(2.0)  # Allow hardwared to detect ignition and transition to onroad

  env = {**os.environ, "KA2_BURN_IN_DURATION_S": str(ONROAD + 30)}
  test_proc = subprocess.Popen(
    [sys.executable, "selfdrive/test/test_ka2_onroad.py", "--can-replay", "--burn-in-test"],
    cwd=BASEDIR, env=env, preexec_fn=os.setsid
  )

  sleep_track(ONROAD, "Onroad Loop Active")

  # --- PHASE 2: CLEAN TEARDOWN & RECOVERY LAYOUT ---
  stop_harness()

  # Wait for hardwared to detect feeder disconnect and complete offroad transition
  # This matches openpilot's DISCONNECT_TIMEOUT (5s) + transition time
  wait_for_offroad()

  run_cmd(["sync"])
  if os.path.exists(drop_node := "/proc/sys/vm/drop_caches"):
    run_cmd(f"echo 3 | sudo tee {drop_node}", shell=True)

  set_gov("schedutil")
  time.sleep(1.0)

  if cycle < MAX_CYCLES:
    sleep_track(OFFROAD, "Offroad Cooldown Rest")
  else:
    print(f"\n🏁 [{get_timestamp()}] Final cycle {MAX_CYCLES} complete.")
