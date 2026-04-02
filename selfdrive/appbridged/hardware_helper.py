import time
import threading
from cereal import log
from system.hardware.ka2.hardware import Ka2

NetworkType = log.DeviceState.NetworkType
NETWORK_TYPES = {
  NetworkType.none: "Offline",
  NetworkType.wifi: "Wi-Fi",
  NetworkType.cell2G: "2G",
  NetworkType.cell3G: "3G",
  NetworkType.cell4G: "4G",
  NetworkType.cell5G: "5G",
  NetworkType.ethernet: "Ethernet",
}

_ka2 = Ka2()
_last_update_time = 0
_cached_type = NetworkType.none
_lock = threading.Lock()

def get_network_type():
  """
  Rate limit applied because direct Ka2 calls via D-Bus can trigger
  malloc 'unaligned fastbin chunk detected' errors. Testing showed
  1 Hz still fails, while 0.5 Hz or less is stable.
  """
  global _last_update_time, _cached_type
  with _lock:
    if (now := time.monotonic()) - _last_update_time >= 2:
      try:
        _cached_type = _ka2.get_network_type()
      except Exception:
        _cached_type = NetworkType.none
      _last_update_time = now
    return NETWORK_TYPES.get(_cached_type, "Unknown")
