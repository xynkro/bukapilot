#!/usr/bin/env python3
import subprocess, time, threading, json
from cereal import log
from system.hardware.ka2.hardware import Ka2

# Rate limit applied because direct D-Bus or mmcli calls can trigger
# malloc 'unaligned fastbin chunk detected' errors. Testing showed
# 1 Hz still fails, while 0.5 Hz or less is stable
QUERY_INTERVAL = 2.0  # seconds maximum 0.5 Hz

NetworkType = log.DeviceState.NetworkType
NETWORK_TYPES = {
  NetworkType.none:   "Offline",
  NetworkType.wifi:   "Wi-Fi",
  NetworkType.cell2G: "2G",
  NetworkType.cell3G: "3G",
  NetworkType.cell4G: "4G LTE",
}

_ka2 = Ka2()
_ka2_last_update = 0
_ka2_cached_type = NetworkType.none
_ka2_lock = threading.Lock()

_sim_last_resp = ""
_sim_last_time = 0
_sim_lock = threading.Lock()

def get_network_type():
  global _ka2_last_update, _ka2_cached_type
  with _ka2_lock:
    if (now := time.monotonic()) - _ka2_last_update >= QUERY_INTERVAL:
      try:
        _ka2_cached_type = _ka2.get_network_type()
      except Exception:
        _ka2_cached_type = NetworkType.none
      _ka2_last_update = now
    return NETWORK_TYPES.get(_ka2_cached_type, "Unknown")

def _run_mmcli(args):
  try:
    return subprocess.run(args, capture_output=True, text=True, timeout=QUERY_INTERVAL).stdout or ""
  except Exception as e:
    return f"ERROR {e}"

def get_sim_status():
  global _sim_last_resp, _sim_last_time
  with _sim_lock:
    if (now := time.monotonic()) - _sim_last_time < QUERY_INTERVAL and _sim_last_resp:
      return _sim_last_resp

    resp = _run_mmcli(["mmcli", "-m", "0", "--output-json"])
    try:
      data = json.loads(resp).get("modem", {})
    except json.JSONDecodeError:
      _sim_last_resp = "Unknown"
      _sim_last_time = now
      return _sim_last_resp

    generic = data.get("generic", {})
    state = generic.get("state", "").lower()
    reason = generic.get("state-failed-reason", "").lower()
    sim_info = generic.get("sim", "--")

    # Immediate return for hard states
    if state == "failed" and reason == "sim-missing":
      _sim_last_resp = "Not inserted"
      _sim_last_time = now
      return _sim_last_resp
    elif state == "failed":
      _sim_last_resp = "Modem error"
      _sim_last_time = now
      return _sim_last_resp
    elif sim_info == "--":
      _sim_last_resp = "Not ready"
      _sim_last_time = now
      return _sim_last_resp

    # Extract other info
    reg_state = data.get("3gpp", {}).get("registration-state", "").lower()
    rat = ", ".join(generic.get("current-capabilities", [])) or ""
    attached = data.get("3gpp", {}).get("packet-service-state", "").lower() == "attached"
    roaming = data.get("3gpp", {}).get("roaming", "")
    signal = generic.get("signal-quality", {}).get("value", "")
    operator = data.get("3gpp", {}).get("operator-name", "")

    if "searching" in reg_state:
      _sim_last_resp = "Searching"
    elif "registering" in reg_state:
      _sim_last_resp = "Registering"
    elif "emergency" in reg_state:
      _sim_last_resp = "Emergency only"
    elif "registered" in reg_state:
      prefix = (
        "4G LTE" if "lte" in rat else
        "3G HSPA" if "hspa" in rat else
        "3G WCDMA" if "wcdma" in rat else
        "3G" if any(x in rat for x in ["umts", "wcdma", "hspa"]) else
        "2G GSM" if "gsm" in rat else
        "2G EDGE" if "edge" in rat else
        "Registered"
      )
      _sim_last_resp = f"{prefix}, {'data attached' if attached else 'no data'}"
    else:
      _sim_last_resp = "Unknown" if not reg_state and not rat else f"Registered, {'data attached' if attached else 'no data'}"

    if extras := [f"{k} {v}" for k, v in (("roaming", roaming), ("signal", signal), ("operator", operator)) if v]:
      _sim_last_resp += ", " + ", ".join(extras)

    _sim_last_time = now
    return _sim_last_resp
