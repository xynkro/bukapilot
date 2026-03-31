#!/usr/bin/env python3
import subprocess
from time import monotonic
from cereal import log

QUERY_INTERVAL = 0.5
_last_resp, _last_time = "", 0
NetworkType = log.DeviceState.NetworkType

def _run_cmd(args):
  try:
    result = subprocess.run(args, capture_output=True, text=True, timeout=QUERY_INTERVAL)
    # Debug: show full qmicli output
    print("DEBUG:", " ".join(args))
    print(result.stdout)
    return result.stdout or ""
  except Exception as e:
    print("DEBUG ERROR:", e)
    return ""

def _get_card_state():
  resp = _run_cmd(["sudo", "qmicli", "-d", "/dev/cdc-wdm0", "--device-open-proxy", "--uim-get-card-status"])
  if "no-atr-received" in resp or "Card state: 'error" in resp:
    return "None"
  return "Inserted"

def get_sim_status():
  global _last_resp, _last_time
  if (now := monotonic()) - _last_time >= QUERY_INTERVAL or not _last_resp:
    card_state = _get_card_state()
    if card_state == "None":
      _last_resp = "None"
    else:
      resp = _run_cmd(["sudo", "qmicli", "-d", "/dev/cdc-wdm0", "--device-open-proxy", "--nas-get-serving-system"])
      gv = lambda k: resp.split(k)[1].split("'")[1] if k in resp else ""
      reg, cs, ps, rat = gv("Registration state:"), gv("CS:"), gv("PS:"), gv("Selected radio access technology:")
      _last_resp = (
        "Inserted, no data" if reg in ("not-registered-searching", "registration-denied") else
        ("Inserted, 4G" if rat == "lte" else
         "Inserted, 3G" if rat in ("umts", "hspa", "wcdma") else
         "Inserted, 2G" if rat in ("gsm", "edge") else
         "Inserted, no data")
          if cs == "attached" or ps == "attached" else "Unknown"
      )
    _last_time = now
  return _last_resp

def get_sim_network_type():
  if (s := get_sim_status()):
    return NetworkType.cell4G if "4G" in s else (
           NetworkType.cell3G if "3G" in s else (
           NetworkType.cell2G if "2G" in s else None))

if __name__ == "__main__":
  print(get_sim_status())
  print(get_sim_network_type())

