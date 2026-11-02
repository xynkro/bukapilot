#!/usr/bin/env python3
import os
import shutil
import threading
from openpilot.system.hardware.hw import Paths
from openpilot.common.swaglog import cloudlog
from openpilot.system.loggerd.config import get_available_bytes, get_available_percent
from openpilot.system.loggerd.uploader import listdir_by_creation
from openpilot.system.loggerd.xattr_cache import getxattr
from openpilot.system.loggerd.memory_pressure import (
  is_memory_pressure_critical,
  should_skip_filesystem_operation,
  handle_memory_pressure,
)

# comma's 5 GB default is sized for their 32-64 GB devices. The KA2's /data partition is
# only 8.2 GB, so a 5 GB reserve is 61% of the disk -- the deleter therefore wiped EVERY
# segment as fast as loggerd wrote it, and /data/media/0/realdata was permanently empty.
# That is why no drive of Caspar's has ever left an rlog to diagnose from.
# 2.5 GB is ~30% of this partition: still enough headroom to rebuild the ~2.0 GB
# safe_staging/finalized tree during an update, while leaving ~1.2 GB for segments.
MIN_BYTES = 2.5 * 1024 * 1024 * 1024
MIN_PERCENT = 10

DELETE_LAST = ['boot', 'crash']

PRESERVE_ATTR_NAME = 'user.preserve'
PRESERVE_ATTR_VALUE = b'1'
PRESERVE_COUNT = 5


def has_preserve_xattr(d: str) -> bool:
  return getxattr(os.path.join(Paths.log_root(), d), PRESERVE_ATTR_NAME) == PRESERVE_ATTR_VALUE


def get_preserved_segments(dirs_by_creation: list[str]) -> set[str]:
  # skip deleting most recent N preserved segments (and their prior segment)
  preserved = set()
  for n, d in enumerate(filter(has_preserve_xattr, reversed(dirs_by_creation))):
    if n == PRESERVE_COUNT:
      break
    date_str, _, seg_str = d.rpartition("--")

    # ignore non-segment directories
    if not date_str:
      continue
    try:
      seg_num = int(seg_str)
    except ValueError:
      continue

    # preserve segment and two prior
    for _seg_num in range(max(0, seg_num - 2), seg_num + 1):
      preserved.add(f"{date_str}--{_seg_num}")

  return preserved


def deleter_thread(exit_event: threading.Event):
  while not exit_event.is_set():
    # Check memory pressure first - skip operations if memory is critical
    if is_memory_pressure_critical():
      handle_memory_pressure(clear_caches=True, clear_fs_cache=False)
      cloudlog.warning("Skipping deleter operations due to critical memory pressure")
      exit_event.wait(5)  # Wait shorter time when memory is critical
      continue

    out_of_bytes = get_available_bytes(default=MIN_BYTES + 1) < MIN_BYTES
    out_of_percent = get_available_percent(default=MIN_PERCENT + 1) < MIN_PERCENT

    if out_of_percent or out_of_bytes:
      # Check memory before doing expensive directory scan
      if should_skip_filesystem_operation():
        cloudlog.warning("Skipping directory scan due to memory pressure")
        exit_event.wait(5)
        continue

      dirs = listdir_by_creation(Paths.log_root())
      preserved_dirs = get_preserved_segments(dirs)

      # remove the earliest directory we can
      for delete_dir in sorted(dirs, key=lambda d: (d in DELETE_LAST, d in preserved_dirs)):
        delete_path = os.path.join(Paths.log_root(), delete_dir)

        # Check memory again before each deletion operation
        if should_skip_filesystem_operation():
          break

        if any(name.endswith(".lock") for name in os.listdir(delete_path)):
          continue

        try:
          cloudlog.info(f"deleting {delete_path}")
          shutil.rmtree(delete_path)
          break
        except OSError:
          cloudlog.exception(f"issue deleting {delete_path}")
      exit_event.wait(.1)
    else:
      exit_event.wait(30)


def main():
  deleter_thread(threading.Event())


if __name__ == "__main__":
  main()
