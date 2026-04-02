import hashlib
import math
import os
import subprocess
import threading
import time
from enum import IntEnum
from functools import cached_property, lru_cache
from pathlib import Path

# Need to tell the C library to be thread-safe for D-Bus
import dbus
import dbus.mainloop.glib
try:
  dbus.mainloop.glib.threads_init()
except Exception:
  pass

from cereal import log
from openpilot.common.gpio import gpio_set, gpio_init, get_irqs_for_action
from openpilot.system.hardware.base import HardwareBase, ThermalConfig, ThermalZone
from openpilot.system.hardware.ka2 import iwlist

NM = 'org.freedesktop.NetworkManager'
NM_CON_ACT = NM + '.Connection.Active'
NM_DEV = NM + '.Device'
NM_DEV_WL = NM + '.Device.Wireless'
NM_DEV_STATS = NM + '.Device.Statistics'
NM_AP = NM + '.AccessPoint'
DBUS_PROPS = 'org.freedesktop.DBus.Properties'

MM = 'org.freedesktop.ModemManager1'
MM_MODEM = MM + ".Modem"
MM_MODEM_SIMPLE = MM + ".Modem.Simple"
MM_SIM = MM + ".Sim"

class MM_MODEM_STATE(IntEnum):
  FAILED        = -1
  UNKNOWN       = 0
  INITIALIZING  = 1
  LOCKED        = 2
  DISABLED      = 3
  DISABLING     = 4
  ENABLING      = 5
  ENABLED       = 6
  SEARCHING     = 7
  REGISTERED    = 8
  DISCONNECTING = 9
  CONNECTING    = 10
  CONNECTED     = 11

class NMMetered(IntEnum):
  NM_METERED_UNKNOWN = 0
  NM_METERED_YES = 1
  NM_METERED_NO = 2
  NM_METERED_GUESS_YES = 3
  NM_METERED_GUESS_NO = 4

TIMEOUT = 0.1
REFRESH_RATE_MS = 1000

# Limit repeated D-Bus-heavy getter paths.
#
# Behaviour:
# - first call is always fresh/live
# - if called slowly enough, continue fetching fresh/live values
# - if called too frequently, return the last successful real value in between
# - no fake placeholder values are introduced purely due to rate limiting
#
# Why:
# Repeated aggressive D-Bus usage can trigger or expose native heap corruption
# bugs in dbus-python / libdbus / GLib stacks, especially under threaded access.
#
# Errors like:
#   malloc(): unaligned fastbin chunk detected
#
# usually indicate native allocator metadata corruption (C-level heap corruption),
# not a normal Python exception. Python itself usually isn't the direct cause, but
# high-frequency threaded D-Bus traffic can be enough to expose it.
DBUS_MIN_INTERVAL = 2.0  # 0.5 Hz

NetworkType = log.DeviceState.NetworkType
NetworkStrength = log.DeviceState.NetworkStrength

# https://developer.gnome.org/ModemManager/unstable/ModemManager-Flags-and-Enumerations.html#MMModemAccessTechnology
MM_MODEM_ACCESS_TECHNOLOGY_UMTS = 1 << 5
MM_MODEM_ACCESS_TECHNOLOGY_LTE = 1 << 14

def sudo_write(val, path):
  data = str(val)
  try:
    with open(path, "w") as f:
      f.write(data)
  except PermissionError:
    if os.system(f"sudo chmod a+w {path}") == 0:
      try:
        with open(path, "w") as f:
          f.write(data)
        return
      except PermissionError:
        os.system(f"sudo sh -c 'echo {data} > {path}'")
  except Exception:
    pass
  finally:
    return

def sudo_read(path: str) -> str:
  try:
    return subprocess.run(["sudo", "cat", path], check=True, capture_output=True, text=True).stdout.strip()
  except Exception:
    return ""

def affine_irq(val, action):
  if not (irqs := get_irqs_for_action(action)):
    print(f"No IRQs found for '{action}'")
    return

  for i in irqs:
    sudo_write(str(val), f"/proc/irq/{i}/smp_affinity_list")

class Ka2(HardwareBase):
  def __init__(self):
    super().__init__()
    # Utilise thread-local storage to provide unique bus connections for each thread.
    self._local = threading.local()
    self._local.bus_pid = None
    self._lock = threading.RLock()

    # Shared cache for rate-limited D-Bus-backed getters.
    #
    # IMPORTANT:
    # Do NOT cache live D-Bus proxy objects here (modem/wlan/wwan/etc).
    # Those can become stale if NetworkManager / ModemManager restarts.
    #
    # Only cache:
    # - plain values
    # - object paths
    # - dict/list/tuple results
    # - enums / strings / ints / bools
    self._dbus_cache = {}
    self._dbus_cache_lock = threading.Lock()

  @property
  def bus(self):
    # Validate process identifier and thread-local state to avoid stale D-Bus handles.
    pid = os.getpid()
    if not hasattr(self._local, 'bus_obj') or self._local.bus_pid != pid:
      self._local.bus_obj = dbus.SystemBus()
      self._local.bus_pid = pid
    return self._local.bus_obj

  @property
  def nm(self):
    return self.bus.get_object(NM, '/org/freedesktop/NetworkManager')

  @property # this should not be cached, in case the modemmanager restarts
  def mm(self):
    return self.bus.get_object(MM, '/org/freedesktop/ModemManager1')

  def _dbus_cached(self, key, fetch_fn):
    """
    Shared rate limiter for D-Bus-heavy getters.

    Rules:
    - first call for a key is always fresh/live
    - if called too frequently, return the last successful real value
    - if enough time has passed, fetch fresh/live again
    - if a refresh fails but an older successful value exists, return the older real value
    - if the very first fetch fails, let the caller handle fallback behaviour
    """
    now = time.monotonic()

    with self._dbus_cache_lock:
      if (entry := self._dbus_cache.get(key)) is not None:
        if (now - entry["ts"]) < DBUS_MIN_INTERVAL:
          return entry["value"]

    try:
      value = fetch_fn()
    except Exception:
      with self._dbus_cache_lock:
        if (entry := self._dbus_cache.get(key)) is not None:
          return entry["value"]
      raise

    with self._dbus_cache_lock:
      self._dbus_cache[key] = {
        "ts": now,
        "value": value,
      }

    return value

  def _get_modem_path(self, *, fresh=False):
    def _fetch():
      objects = self.mm.GetManagedObjects(dbus_interface="org.freedesktop.DBus.ObjectManager", timeout=TIMEOUT)
      if (paths := list(objects.keys())):
        return str(paths[0])
      return None

    try:
      return _fetch() if fresh else self._dbus_cached("modem_path", _fetch)
    except Exception:
      return None

  def _get_wlan_path(self):
    def _fetch():
      path = self.nm.GetDeviceByIpIface('wlan0', dbus_interface=NM, timeout=TIMEOUT)
      return str(path) if path else None

    try:
      return self._dbus_cached("wlan_path", _fetch)
    except Exception:
      return None

  def _get_wwan_path(self):
    def _fetch():
      path = self.nm.GetDeviceByIpIface('wwan0', dbus_interface=NM, timeout=TIMEOUT)
      return str(path) if path else None

    try:
      return self._dbus_cached("wwan_path", _fetch)
    except Exception:
      return None

  def get_modem(self):
    try:
      if (path := self._get_modem_path()):
        return self.bus.get_object(MM, path)
    except Exception:
      pass
    return None

  def get_wlan(self):
    try:
      if (path := self._get_wlan_path()):
        return self.bus.get_object(NM, path)
    except Exception:
      pass
    return None

  def get_wwan(self):
    try:
      if (path := self._get_wwan_path()):
        return self.bus.get_object(NM, path)
    except Exception:
      pass
    return None

  def _get_modem_fresh(self):
    try:
      if (path := self._get_modem_path(fresh=True)):
        return self.bus.get_object(MM, path)
    except Exception:
      pass
    return None

  def _get_sim_info_fresh(self):
    try:
      with self._lock:
        if not (modem := self._get_modem_fresh()): return None
        sim_path = modem.Get(MM_MODEM, 'Sim', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)

        if sim_path == "/":
          return {
            'sim_id': '',
            'mcc_mnc': None,
            'network_type': ["Unknown"],
            'sim_state': ["ABSENT"],
            'data_connected': False
          }

        sim = self.bus.get_object(MM, str(sim_path))
        return {
          'sim_id': str(sim.Get(MM_SIM, 'SimIdentifier', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)),
          'mcc_mnc': str(sim.Get(MM_SIM, 'OperatorIdentifier', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)),
          'network_type': ["Unknown"],
          'sim_state': ["READY"],
          'data_connected': modem.Get(MM_MODEM, 'State', dbus_interface=DBUS_PROPS, timeout=TIMEOUT) == MM_MODEM_STATE.CONNECTED,
        }
    except Exception:
      return None

  def get_os_version(self):
    with open("/VERSION") as f:
      return f.read().strip()

  def get_device_type(self):
    return "ka2"

  # ka2 sound card is always online
  def get_sound_card_online(self):
    return True

  def reboot(self, reason=None):
    subprocess.check_output(["sudo", "reboot"])

  def uninstall(self):
    Path("/data/__system_reset__").touch()
    os.sync()
    self.reboot()

  def get_current_power_draw(self):
    # Same I2C power monitor (e.g. ina3221) at 0-0040 as in power_monitor.py; power1_input is microwatts
    return HardwareBase.read_param_file(
      "/sys/bus/i2c/devices/0-0040/hwmon/hwmon1/power1_input", int, 0
    ) / 1e6

  def get_som_power_draw(self):
    # KA2 has no separate SoM power rail sensor (unlike TICI BMS); total draw is from get_current_power_draw
    return 0

  def get_screen_brightness(self):
    return 0

  def set_screen_brightness(self, percentage):
    pass

  def get_serial(self):
    return subprocess.check_output("grep 'Serial' /proc/cpuinfo | sed 's/.*: //'", shell=True, text=True).strip()

  def get_network_type(self):
    def _fetch():
      with self._lock:
        try:
          if not (p_conn := self.nm.Get(NM, 'PrimaryConnection', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)):
            return NetworkType.none

          primary_connection = self.bus.get_object(NM, str(p_conn))
          primary_type = primary_connection.Get(NM_CON_ACT, 'Type', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)

          if primary_type == '802-3-ethernet':
            return NetworkType.ethernet
          if primary_type == '802-11-wireless':
            return NetworkType.wifi

          active_connections = self.nm.Get(NM, 'ActiveConnections', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)
          for conn in active_connections:
            c = self.bus.get_object(NM, str(conn))
            if c.Get(NM_CON_ACT, 'Type', dbus_interface=DBUS_PROPS, timeout=TIMEOUT) == 'gsm':
              if (modem := self.get_modem()):
                access_t = int(modem.Get(MM_MODEM, 'AccessTechnologies', dbus_interface=DBUS_PROPS, timeout=TIMEOUT))
                if access_t & MM_MODEM_ACCESS_TECHNOLOGY_LTE:
                  return NetworkType.cell4G
                if access_t & MM_MODEM_ACCESS_TECHNOLOGY_UMTS:
                  return NetworkType.cell3G
                return NetworkType.cell2G
        except Exception:
          pass
      return NetworkType.none

    try:
      return self._dbus_cached("get_network_type", _fetch)
    except Exception:
      return NetworkType.none

  def get_sim_info(self):
    def _fetch():
      with self._lock:
        try:
          if not (modem := self.get_modem()): return None
          sim_path = modem.Get(MM_MODEM, 'Sim', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)

          if sim_path == "/":
            return {
              'sim_id': '',
              'mcc_mnc': None,
              'network_type': ["Unknown"],
              'sim_state': ["ABSENT"],
              'data_connected': False
            }

          sim = self.bus.get_object(MM, str(sim_path))
          return {
            'sim_id': str(sim.Get(MM_SIM, 'SimIdentifier', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)),
            'mcc_mnc': str(sim.Get(MM_SIM, 'OperatorIdentifier', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)),
            'network_type': ["Unknown"],
            'sim_state': ["READY"],
            'data_connected': modem.Get(MM_MODEM, 'State', dbus_interface=DBUS_PROPS, timeout=TIMEOUT) == MM_MODEM_STATE.CONNECTED,
          }
        except Exception:
          return None

    try:
      return self._dbus_cached("get_sim_info", _fetch)
    except Exception:
      return None

  def get_imei(self, slot):
    if slot != 0:
      return ""
    try:
      def _fetch():
        with self._lock:
          if (modem := self.get_modem()):
            return str(modem.Get(MM_MODEM, "EquipmentIdentifier", dbus_interface=DBUS_PROPS, timeout=TIMEOUT))
        raise RuntimeError("No modem IMEI available")

      return self._dbus_cached("get_imei_slot0", _fetch)
    except Exception:
      pass
    # Generate fake 15 digit imei from wlan0 mac address
    mac = subprocess.getoutput("cat /sys/class/net/wlan0/address")
    return hashlib.sha256(mac.replace(":", "").replace("-", "").encode()).hexdigest()[:15]

  def get_network_info(self):
    def _fetch():
      with self._lock:
        try:
          if not (modem := self.get_modem()): return None
          info = modem.Command("AT+QNWINFO", math.ceil(TIMEOUT), dbus_interface=MM_MODEM, timeout=TIMEOUT)
          extra = modem.Command('AT+QENG="servingcell"', math.ceil(TIMEOUT), dbus_interface=MM_MODEM, timeout=TIMEOUT)
          state = modem.Get(MM_MODEM, 'State', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)
        except Exception:
          return None

        if info and info.startswith('+QNWINFO: '):
          info_list = info.replace('+QNWINFO: ', '').replace('"', '').split(',')
          if len(info_list) == 4:
            return {
              'technology': info_list[0],
              'operator': info_list[1],
              'band': info_list[2],
              'channel': int(info_list[3]),
              'extra': "" if extra is None else extra.replace('+QENG: "servingcell",', '').replace('"', ''),
              'state': "" if state is None else MM_MODEM_STATE(state).name,
            }
      return None

    try:
      return self._dbus_cached("get_network_info", _fetch)
    except Exception:
      return None

  def parse_strength(self, percentage):
    if percentage < 25:
      return NetworkStrength.poor
    if percentage < 50:
      return NetworkStrength.moderate
    if percentage < 75:
      return NetworkStrength.good
    return NetworkStrength.great

  def get_network_strength(self, network_type):
    def _fetch():
      network_strength = NetworkStrength.unknown
      with self._lock:
        try:
          if network_type == NetworkType.wifi:
            if (wlan := self.get_wlan()) and (ap_path := wlan.Get(NM_DEV_WL, 'ActiveAccessPoint', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)) != "/":
              active_ap = self.bus.get_object(NM, str(ap_path))
              strength = int(active_ap.Get(NM_AP, 'Strength', dbus_interface=DBUS_PROPS, timeout=TIMEOUT))
              network_strength = self.parse_strength(strength)
          elif network_type != NetworkType.none:  # Cellular
            if (modem := self.get_modem()):
              strength = int(modem.Get(MM_MODEM, 'SignalQuality', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)[0])
              network_strength = self.parse_strength(strength)
        except Exception:
          pass
      return network_strength

    try:
      return self._dbus_cached(f"get_network_strength:{int(network_type)}", _fetch)
    except Exception:
      return NetworkStrength.unknown

  def get_network_metered(self, network_type) -> bool:
    def _fetch():
      with self._lock:
        try:
          if not (p_path := self.nm.Get(NM, 'PrimaryConnection', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)):
            return super(Ka2, self).get_network_metered(network_type)

          primary_connection = self.bus.get_object(NM, str(p_path))
          for dev in primary_connection.Get(NM_CON_ACT, 'Devices', dbus_interface=DBUS_PROPS, timeout=TIMEOUT):
            dev_obj = self.bus.get_object(NM, str(dev))
            metered_prop = dev_obj.Get(NM_DEV, 'Metered', dbus_interface=DBUS_PROPS, timeout=TIMEOUT)

            if network_type == NetworkType.wifi and metered_prop in [NMMetered.NM_METERED_YES, NMMetered.NM_METERED_GUESS_YES]:
              return True
            if network_type in [NetworkType.cell2G, NetworkType.cell3G, NetworkType.cell4G, NetworkType.cell5G] and metered_prop == NMMetered.NM_METERED_NO:
              return False
        except Exception:
          pass
      return super(Ka2, self).get_network_metered(network_type)

    try:
      return self._dbus_cached(f"get_network_metered:{int(network_type)}", _fetch)
    except Exception:
      return super().get_network_metered(network_type)

  def get_modem_version(self):
    def _fetch():
      with self._lock:
        try:
          if (modem := self.get_modem()):
            return str(modem.Get(MM_MODEM, 'Revision', dbus_interface=DBUS_PROPS, timeout=TIMEOUT))
        except Exception:
          pass
      return None

    try:
      return self._dbus_cached("get_modem_version", _fetch)
    except Exception:
      return None

  def get_modem_nv(self):
    timeout = 0.2  # Default timeout is too short
    files = (
      '/nv/item_files/modem/mmode/ue_usage_setting',
      '/nv/item_files/ims/IMS_enable',
      '/nv/item_files/modem/mmode/sms_only',
    )

    def _fetch():
      with self._lock:
        try:
          if (modem := self.get_modem()):
            return {fn: modem.Command(f'AT+QNVFR="{fn}"', math.ceil(timeout), dbus_interface=MM_MODEM, timeout=timeout) for fn in files}
        except Exception:
          pass
      return None

    try:
      return self._dbus_cached("get_modem_nv", _fetch)
    except Exception:
      return None

  def get_modem_temperatures(self):
    timeout = 0.2  # Default timeout is too short

    def _fetch():
      with self._lock:
        try:
          if (modem := self.get_modem()):
            temps = modem.Command("AT+QTEMP", math.ceil(timeout), dbus_interface=MM_MODEM, timeout=timeout)
            return list(map(int, temps.split(' ')[1].split(',')))
        except Exception:
          pass
      return []

    try:
      return self._dbus_cached("get_modem_temperatures", _fetch)
    except Exception:
      return []

  def shutdown(self):
    os.system("sudo poweroff")

  def get_thermal_config(self):
    return ThermalConfig(
      cpu=[ThermalZone("bigcore0-thermal", 1000), ThermalZone("bigcore1-thermal", 1000)],
      gpu=[ThermalZone("gpu-thermal", 1000), ThermalZone("npu-thermal", 1000)],
      memory=ThermalZone("center-thermal", 1000),
      pmic=[ThermalZone("soc-thermal", 1000), ThermalZone("center-thermal", 1000)],
    )

  def set_power_save(self, powersave_enabled):
    # *** CPU config ***

    # Offline big cluster, leave core 4 online for boardd
    for i in range(5, 8):
      sudo_write('0' if powersave_enabled else '1', f'/sys/devices/system/cpu/cpu{i}/online')

    for n in ('0', '4'):
      sudo_write('ondemand' if powersave_enabled else 'performance', f'/sys/devices/system/cpu/cpufreq/policy{n}/scaling_governor')

    # *** GPU (Mali): 300 MHz + simple_ondemand in power save, else 1000 MHz + performance ***
    _gpu_devfreq = "/sys/class/devfreq/fb000000.gpu"
    if os.path.isfile(f"{_gpu_devfreq}/governor"):
      if powersave_enabled:
        sudo_write("300000000", f"{_gpu_devfreq}/max_freq")
        sudo_write("simple_ondemand", f"{_gpu_devfreq}/governor")
      else:
        sudo_write("1000000000", f"{_gpu_devfreq}/max_freq")
        sudo_write("performance", f"{_gpu_devfreq}/governor")

    # *** IRQ config ***
    pass

    # boardd core
    #affine_irq(4, "spi_geni")          # SPI
    #affine_irq(4, "xhci-hcd:usb3")     # aux panda USB (or potentially anything else on USB)
    #if "tici" in self.get_device_type():
    #  affine_irq(4, "xhci-hcd:usb1")   # internal panda USB (also modem)

    # GPU
    #affine_irq(5, "kgsl-3d0")

    # camerad core
    #camera_irqs = ("cci", "cpas_camnoc", "cpas-cdm", "csid", "ife", "csid-lite", "ife-lite")
    #for n in camera_irqs:
    #  affine_irq(5, n)

  def get_gpu_usage_percent(self):
    # Mali devfreq exposes load as "percent@freqHz" (e.g. "0@300000000Hz")
    try:
      with open("/sys/class/devfreq/fb000000.gpu/load", "r") as f:
        s = f.read().strip()
      return int(float(s.split("@")[0])) if "@" in s else int(float(s))
    except Exception:
      return 0

  def get_npu_usage_percent(self):
    try:
      if (npu_load := sudo_read("/sys/kernel/debug/rknpu/load")):
        return [int(x.split('%')[0]) for x in npu_load.split() if '%' in x]
    except Exception:
      pass
    return [0, 0, 0]

  def initialize_hardware(self):
    # Allow thermald to write engagement status to kmsg
    os.system("sudo chmod a+w /dev/kmsg")

    # Ensure fan gpio is enabled so fan runs until shutdown, also turned on at boot by the ABL
    # TODO gpio_init(GPIO.SOM_ST_IO, True)
    # TODO gpio_set(GPIO.SOM_ST_IO, 1)

    # *** IRQ config ***

    # Mask off big cluster from default affinity
    sudo_write("f", "/proc/irq/default_smp_affinity")

    # Move these off the default core
    #affine_irq(1, "msm_drm")   # display
    #affine_irq(1, "msm_vidc")  # encoders
    #affine_irq(1, "i2c_geni")  # sensors

    # Initialise cpu, ddr and npu governors
    # TODO see if cpu and ddr needed
    sudo_write("userspace", "/sys/class/devfreq/fdab0000.npu/governor")
    sudo_write("1000000000", "/sys/class/devfreq/fdab0000.npu/userspace/set_freq")

    sudo_write("userspace", "/sys/class/devfreq/dmc/governor")
    sudo_write("2112000000", "/sys/class/devfreq/dmc/userspace/set_freq")

  def configure_modem(self):
    with self._lock:
      # Use a fresh direct SIM read here instead of the shared rate-limited getter,
      # since modem configuration is a startup/control path and should use current state.
      mcc_mnc = (self._get_sim_info_fresh() or {}).get('mcc_mnc') or ''

      if os.path.isfile(wwan0_setup := "/usr/kommu/lte/wwan0-setup.sh"):
        os.system(f"bash {wwan0_setup} {mcc_mnc}")

      if (modem := self._get_modem_fresh()):
        cmds = [
          # Configure modem as data-centric
          'AT+QNVW=5280,0,"0102000000000000"',
          'AT+QNVFW="/nv/item_files/ims/IMS_enable",00',
          'AT+QNVFW="/nv/item_files/modem/mmode/ue_usage_setting",01',
        ]

        for cmd in cmds:
          try:
            modem.Command(cmd, math.ceil(TIMEOUT), dbus_interface=MM_MODEM, timeout=TIMEOUT)
          except Exception:
            pass

      # Fallback: directly remove default route if modem connects with one
      os.system("sudo ip route del default dev wwan0 2>/dev/null || true")

  def get_networks(self):
    r = {}

    if (wlan := iwlist.scan()) is not None:
      r['wlan'] = wlan

    if (lte := self.get_network_info()) and 'LTE' in (ex := lte['extra']):
      ex_list = ex.split(',')
      try:
        r['lte'] = [{
          "mcc": int(ex_list[3]),
          "mnc": int(ex_list[4]),
          "cid": int(ex_list[5], 16),
          "nmr": [{"pci": int(ex_list[6]), "earfcn": int(ex_list[7])}],
        }]
      except (ValueError, IndexError):
        pass

    return r

  def get_modem_data_usage(self):
    def _fetch():
      with self._lock:
        try:
          if not (wwan := self.get_wwan()): return -1, -1

          # Ensure refresh rate is set so values do not go stale
          current_refresh = int(wwan.Get(NM_DEV_STATS, 'RefreshRateMs', dbus_interface=DBUS_PROPS, timeout=TIMEOUT))
          if current_refresh != REFRESH_RATE_MS:
            wwan.Set(NM_DEV_STATS, 'RefreshRateMs', dbus.UInt32(REFRESH_RATE_MS), dbus_interface=DBUS_PROPS, timeout=TIMEOUT)

          tx = int(wwan.Get(NM_DEV_STATS, 'TxBytes', dbus_interface=DBUS_PROPS, timeout=TIMEOUT))
          rx = int(wwan.Get(NM_DEV_STATS, 'RxBytes', dbus_interface=DBUS_PROPS, timeout=TIMEOUT))
          return tx, rx
        except Exception:
          return -1, -1

    try:
      return self._dbus_cached("get_modem_data_usage", _fetch)
    except Exception:
      return -1, -1

  def has_internal_panda(self):
    return True

  def reset_internal_panda(self):
    #gpio_init(GPIO.STM_RST_N, True)
    #gpio_set(GPIO.STM_RST_N, 1)
    #time.sleep(1)
    #gpio_set(GPIO.STM_RST_N, 0)
    pass

  def recover_internal_panda(self):
    #gpio_init(GPIO.STM_RST_N, True)
    #gpio_init(GPIO.STM_BOOT0, True)
    #gpio_set(GPIO.STM_RST_N, 1)
    #gpio_set(GPIO.STM_BOOT0, 1)
    #time.sleep(0.5)
    #gpio_set(GPIO.STM_RST_N, 0)
    #time.sleep(0.5)
    #gpio_set(GPIO.STM_BOOT0, 0)
    pass

  def booted(self):
    return True

if __name__ == "__main__":
  t = Ka2()
  t.configure_modem()
  t.initialize_hardware()
  t.set_power_save(False)
