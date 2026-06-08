#include "system/camerad/cameras/camera_rk.h"

#include <poll.h>
#include <sys/ioctl.h>

#include <algorithm>
#include <cassert>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <string>
#include <vector>

#include "media/cam_defs.h"
#include "media/cam_isp.h"
#include "media/cam_isp_ife.h"
#include "media/cam_req_mgr.h"
#include "media/cam_sensor_cmn_header.h"
#include "media/cam_sync.h"
#include "third_party/linux/include/v4l2-controls.h"
#include "common/swaglog.h"
#include "common/timing.h"

// Special defined
#define V4L2_CID_X3C_SENSOR_TEMPERATURE (V4L2_CID_USER_BASE + 0x100)

extern ExitHandler do_exit;

static const bool env_disable_wide_road = (getenv("DISABLE_WIDE_ROAD") != nullptr);
static const bool env_disable_road = (getenv("DISABLE_ROAD") != nullptr);
static const bool env_disable_driver = (getenv("DISABLE_DRIVER") != nullptr);
static const bool env_log_raw_frames = (getenv("LOG_RAW_FRAMES") != nullptr);
static const bool env_debug_camera = (getenv("DEBUG_CAMERA") != nullptr);

#define DEBUG_LOG(fmt, ...) do { if (env_debug_camera) { LOGD(fmt, ##__VA_ARGS__); } } while(0)
#define DEBUG_LOG_ERR(fmt, ...) do { if (env_debug_camera) { LOGE(fmt, ##__VA_ARGS__); } } while(0)

static inline bool read_ctrl_fd(int fd, uint32_t id, int *out) {
  struct v4l2_control c = {};
  c.id = id;
  if (ioctl(fd, VIDIOC_G_CTRL, &c) < 0) return false;
  *out = c.value;
  return true;
}

void CameraState::camera_map_bufs(MultiCameraState *s) {
  DEBUG_LOG("camera_map_bufs: camera_num=%d starting", camera_num);
  int exported_count = 0;
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    DEBUG_LOG("camera_map_bufs: mapping buffer %d/%d", i, FRAME_BUF_COUNT);
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.index = i;
    v4l_buf.length = 1; //FMT_NUM_PLANES
    v4l_buf.m.planes = planes;

    if (ioctl(video_fd, VIDIOC_QUERYBUF, &v4l_buf) < 0) {
      DEBUG_LOG_ERR("camera_map_bufs: VIDIOC_QUERYBUF FAILED idx=%d errno=%d '%s'", i, errno, strerror(errno));
    }
    assert(ioctl(video_fd, VIDIOC_QUERYBUF, &v4l_buf) >= 0);

    buf.camera_bufs[i].mmap_len = v4l_buf.m.planes[0].length;
    buf.camera_bufs[i].len = v4l_buf.m.planes[0].length;
    buf.camera_bufs[i].fd = -1;
    DEBUG_LOG("camera_map_bufs: mmap buffer %d len=%u", i, v4l_buf.m.planes[0].length);
    buf.camera_bufs[i].addr = mmap(NULL, v4l_buf.m.planes[0].length,
                                  PROT_READ | PROT_WRITE,
                                  MAP_SHARED,
                                  video_fd, v4l_buf.m.planes[0].m.mem_offset);
    if (buf.camera_bufs[i].addr == MAP_FAILED) {
      DEBUG_LOG_ERR("camera_map_bufs: mmap FAILED idx=%d errno=%d '%s'", i, errno, strerror(errno));
    }
    assert(buf.camera_bufs[i].addr != MAP_FAILED);
    buf.camera_bufs[i].init_yuv(buf.rgb_width, buf.rgb_height, buf.rgb_width, (size_t)buf.rgb_width * buf.rgb_height);

    if (rk_zerocopy_requested) {
      struct v4l2_exportbuffer exp = {};
      exp.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
      exp.index = i;
      exp.plane = 0;
      exp.flags = O_CLOEXEC | O_RDWR;
      if (ioctl(video_fd, VIDIOC_EXPBUF, &exp) == 0) {
        buf.camera_bufs[i].fd = exp.fd;
        buf.camera_bufs[i].frame_id_in_buf = false;
        exported_count++;
        DEBUG_LOG("camera_map_bufs: exported buffer idx=%d fd=%d", i, exp.fd);
      } else {
        LOGW("camera %d: VIDIOC_EXPBUF failed idx=%d errno=%d '%s', disabling rk zerocopy",
             camera_num, i, errno, strerror(errno));
      }
    }
  }

  rk_zerocopy_active = rk_zerocopy_requested && (exported_count == FRAME_BUF_COUNT);
  if (!rk_zerocopy_active) {
    DEBUG_LOG("camera_map_bufs: rk_zerocopy NOT active (requested=%d exported=%d/%d)", 
              rk_zerocopy_requested, exported_count, FRAME_BUF_COUNT);
    for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
      if (buf.camera_bufs[i].fd >= 0) {
        close(buf.camera_bufs[i].fd);
        buf.camera_bufs[i].fd = -1;
      }
      buf.camera_bufs[i].frame_id_in_buf = true;
    }
  } else {
    LOGD("camera %d: rk zerocopy enabled with %d exported buffers", camera_num, exported_count);
  }
  DEBUG_LOG("camera_map_bufs: camera_num=%d DONE exported=%d", camera_num, exported_count);
}

void CameraState::camera_init(MultiCameraState *s, VisionIpcServer * v, cl_device_id device_id, cl_context ctx, VisionStreamType yuv_type) {
  if (!enabled) return;
  rk_zerocopy_requested = (getenv("CAMERAD_RK_ZEROCOPY") != nullptr);
  rk_zerocopy_active = false;

  DEBUG_LOG("camera_init: camera_num=%d starting", camera_num);

  LOG("-- Setting camera ctrls");

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  fmt.fmt.pix.width = 1920;
  fmt.fmt.pix.height = 1200;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_NV12;
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  DEBUG_LOG("camera_init: calling VIDIOC_S_FMT video_fd=%d", video_fd.fd_);
  if (ioctl(video_fd, VIDIOC_S_FMT, &fmt) < 0) {
    int err = errno;
    int vfd = video_fd;
    DEBUG_LOG_ERR("camera_init: VIDIOC_S_FMT FAILED camera_num=%d fd=%d errno=%d '%s'", camera_num, vfd, err, strerror(errno));
    LOGE("camera %d: VIDIOC_S_FMT failed on fd %d (errno=%d '%s'), disabling camera",
         camera_num, vfd, err, strerror(err));
    enabled = false;
    return;
  }
  DEBUG_LOG("camera_init: VIDIOC_S_FMT success");

  memset(&req, 0, sizeof(req));
  req.count = FRAME_BUF_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  req.memory = V4L2_MEMORY_MMAP;
  DEBUG_LOG("camera_init: calling VIDIOC_REQBUFS count=%d", FRAME_BUF_COUNT);
  if (ioctl(video_fd, VIDIOC_REQBUFS, &req) < 0) {
    DEBUG_LOG_ERR("camera_init: VIDIOC_REQBUFS FAILED errno=%d '%s'", errno, strerror(errno));
  }
  LOGD("camera %d: VIDIOC_REQBUFS req.count=%d got.count=%d (asserting success)", camera_num, FRAME_BUF_COUNT, req.count);
  assert(ioctl(video_fd, VIDIOC_REQBUFS, &req) >= 0);

  DEBUG_LOG("camera_init: calling buf.init");
  buf.init(device_id, ctx, this, v, FRAME_BUF_COUNT, yuv_type);
  DEBUG_LOG("camera_init: calling camera_map_bufs");
  camera_map_bufs(s);
  DEBUG_LOG("camera_init: calling setupVipcBuffers rk_zerocopy_active=%d", rk_zerocopy_active);
  buf.setupVipcBuffers(rk_zerocopy_active);
  DEBUG_LOG("camera_init: camera_num=%d DONE", camera_num);
}

void CameraState::camera_open(MultiCameraState *multi_cam_state_, int camera_num_, bool enabled_) {
  multi_cam_state = multi_cam_state_;
  camera_num = camera_num_;
  enabled = enabled_;
  if (!enabled) return;

  DEBUG_LOG("camera_open: camera_num=%d starting", camera_num);

  LOG("-- Setting camera ctrls");
  char device[32];

  // ctrl is at subdev 2,7,12
  snprintf(device, sizeof(device), "/dev/v4l-subdev%d", camera_num * 5 + 2);
  DEBUG_LOG("camera_open: opening ctrl device %s", device);
  ctrl_fd = open(device, O_RDWR);
  if (ctrl_fd < 0) {
    DEBUG_LOG_ERR("camera_open: FAILED to open ctrl_fd errno=%d '%s'", errno, strerror(errno));
    assert(ctrl_fd >= 0);
  }
  DEBUG_LOG("camera_open: ctrl_fd=%d opened", ctrl_fd.fd_);

  // set vflip = 1 to all cameras
  ctrl.id = V4L2_CID_HFLIP;
  ctrl.value = 0;
  if (ioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    DEBUG_LOG_ERR("camera_open: VIDIOC_S_CTRL HFLIP failed errno=%d '%s'", errno, strerror(errno));
  }
  // set vflip = 1 to all cameras
  ctrl.id = V4L2_CID_VFLIP;
  ctrl.value = 1;
  if (ioctl(ctrl_fd, VIDIOC_S_CTRL, &ctrl) < 0) {
    DEBUG_LOG_ERR("camera_open: VIDIOC_S_CTRL VFLIP failed errno=%d '%s'", errno, strerror(errno));
  }

  DEBUG_LOG("camera_open: opening video device for camera_num=%d", camera_num);
  video_fd = open_v4l_by_name_and_index("rkisp_mainpath", camera_num);
  DEBUG_LOG("camera_open: video_fd=%d for camera_num=%d", video_fd.fd_, camera_num);
  if (video_fd < 0) {
    DEBUG_LOG_ERR("camera_open: FAILED to open video_fd errno=%d '%s'", errno, strerror(errno));
  }
  assert(video_fd >= 0);
}

void CameraState::stream_start() {
  if (!enabled) {
    DEBUG_LOG("stream_start: camera_num=%d SKIPPED (not enabled)", camera_num);
    return;
  }
  // start v4l2 buffer queue
  LOG("-- Start Queueing V4L2 buffers");
  DEBUG_LOG("stream_start: camera_num=%d queueing %d buffers", camera_num, FRAME_BUF_COUNT);
  for (int i = 0; i < FRAME_BUF_COUNT; ++i) {
    memset(&v4l_buf, 0, sizeof(v4l_buf));
    memset(planes, 0, sizeof(planes));
    v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    v4l_buf.memory = V4L2_MEMORY_MMAP;
    v4l_buf.length = 1;
    v4l_buf.m.planes = planes;
    v4l_buf.index = i;
    if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
      LOGE("camera %d: VIDIOC_QBUF failed during stream start idx=%d errno=%d '%s' (mode=%s)",
           camera_num, i, errno, strerror(errno), "mmap");
      enabled = false;
      return;
    }
    DEBUG_LOG("stream_start: QBUF idx=%d done", i);
  }

  DEBUG_LOG("stream_start: camera_num=%d calling VIDIOC_STREAMON", camera_num);
  // start streaming
  if (ioctl(video_fd, VIDIOC_STREAMON, &fmt.type) < 0) {
    DEBUG_LOG_ERR("stream_start: VIDIOC_STREAMON FAILED camera_num=%d errno=%d '%s'", camera_num, errno, strerror(errno));
    LOGE("camera %d: VIDIOC_STREAMON failed errno=%d '%s'", camera_num, errno, strerror(errno));
    enabled = false;
  } else {
    DEBUG_LOG("stream_start: VIDIOC_STREAMON SUCCESS camera_num=%d", camera_num);
  }
}

void CameraState::dequeue_buf() {
  if (!enabled) {
    DEBUG_LOG("dequeue_buf: camera_num=%d SKIPPED (not enabled)", camera_num);
    return;
  }

  memset(&v4l_buf, 0, sizeof(v4l_buf));
  memset(planes, 0, sizeof(planes));
  v4l_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
  v4l_buf.memory = V4L2_MEMORY_MMAP;
  v4l_buf.length = 1;
  v4l_buf.m.planes = planes;
  if (ioctl(video_fd, VIDIOC_DQBUF, &v4l_buf) < 0) {
    DEBUG_LOG_ERR("dequeue_buf: camera_num=%d VIDIOC_DQBUF FAILED errno=%d '%s'", camera_num, errno, strerror(errno));
    LOGE("camera %d: VIDIOC_DQBUF failed errno=%d '%s'", camera_num, errno, strerror(errno));
    enabled = false;
    return;
  }
  DEBUG_LOG("dequeue_buf: camera_num=%d idx=%d seq=%d", camera_num, v4l_buf.index, v4l_buf.sequence);

  const int idx = v4l_buf.index;
  FrameMetadata &md = buf.camera_bufs_metadata[idx];

  // Defaults used when control reads are unavailable.
  md.integ_lines = 0;
  md.gain = 1.0f;
  if (md.sensor_temp_c == 0.0f) md.sensor_temp_c = -999.0f;

  int exposure_val = 0, gain_val_raw = 0, analog_gain_val = 0;
  bool have_exposure = false, have_gain = false, have_analog = false;

  if (ext_ctrl_supported) {
    struct v4l2_ext_control ext_ctrls[3] = {};
    struct v4l2_ext_controls ext = {};
    ext.which = V4L2_CTRL_WHICH_CUR_VAL;
    ext.count = 3;
    ext.controls = ext_ctrls;
    ext_ctrls[0].id = V4L2_CID_EXPOSURE;
    ext_ctrls[1].id = V4L2_CID_GAIN;
    ext_ctrls[2].id = V4L2_CID_ANALOGUE_GAIN;

    if (ioctl(ctrl_fd, VIDIOC_G_EXT_CTRLS, &ext) == 0) {
      exposure_val = ext_ctrls[0].value;
      gain_val_raw = ext_ctrls[1].value;
      analog_gain_val = ext_ctrls[2].value;
      have_exposure = true;
      have_gain = gain_val_raw > 0;
      have_analog = analog_gain_val > 0;
    } else if (errno == EINVAL || errno == ENOTTY) {
      ext_ctrl_supported = false;
    }
  }

  if (!ext_ctrl_supported) {
    have_exposure = read_ctrl_fd(ctrl_fd, V4L2_CID_EXPOSURE, &exposure_val);
    have_gain = read_ctrl_fd(ctrl_fd, V4L2_CID_GAIN, &gain_val_raw) && gain_val_raw > 0;
    have_analog = read_ctrl_fd(ctrl_fd, V4L2_CID_ANALOGUE_GAIN, &analog_gain_val) && analog_gain_val > 0;
  }

  if (have_exposure) {
    md.integ_lines = exposure_val;
  }
  if (have_gain) {
    md.gain = static_cast<float>(gain_val_raw);
  } else if (have_analog) {
    md.gain = (analog_gain_val >= 65536) ? (analog_gain_val / 65536.0f) : static_cast<float>(analog_gain_val);
    if (md.gain < 0.01f) md.gain = 1.0f;
  } else {
    static bool gain_warned = false;
    if (!gain_warned) {
      gain_warned = true;
      LOGW("V4L2 gain readback not supported on this subdev (AE likely controlled by rkaiq); logging gain=1.0");
    }
  }

  // Temperature changes slowly, so avoid paying this ioctl every frame.
  if (v4l_buf.sequence % temp_poll_divider == 0) {
    int temp_raw = 0;
    if (read_ctrl_fd(ctrl_fd, V4L2_CID_X3C_SENSOR_TEMPERATURE, &temp_raw)) {
      md.sensor_temp_c = temp_raw / 100.0f;
    } else {
      md.sensor_temp_c = -999.0f;
    }
  }

  md.frame_id = v4l_buf.sequence;
  md.request_id = v4l_buf.sequence;
  cap_time = static_cast<uint64_t>(v4l_buf.timestamp.tv_sec * 1000000000 + v4l_buf.timestamp.tv_usec * 1000);
  md.timestamp_sof = cap_time;
  md.timestamp_eof = cap_time;

  buf.queue(idx);
  DEBUG_LOG("dequeue_buf: camera_num=%d buf.queue idx=%d seq=%d ts=%lu", camera_num, idx, v4l_buf.sequence, (unsigned long)cap_time);

  if (ioctl(video_fd, VIDIOC_QBUF, &v4l_buf) < 0) {
    DEBUG_LOG_ERR("dequeue_buf: camera_num=%d VIDIOC_QBUF post-dequeue FAILED errno=%d '%s'", camera_num, errno, strerror(errno));
    LOGE("camera %d: VIDIOC_QBUF failed post-dequeue errno=%d '%s'", camera_num, errno, strerror(errno));
    enabled = false;
    return;
  }
  DEBUG_LOG("dequeue_buf: camera_num=%d QBUF done seq=%d COMPLETE", camera_num, v4l_buf.sequence);
}

void cameras_init(VisionIpcServer *v, MultiCameraState *s, cl_device_id device_id, cl_context ctx) {
  LOG("-- Initializing cameras");
  LOGD("cameras_init: starting");
  LOGD("cameras_init: driver_cam");
  s->driver_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_DRIVER);
  LOGD("cameras_init: road_cam");
  s->road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_ROAD);
  LOGD("cameras_init: wide_road_cam");
  s->wide_road_cam.camera_init(s, v, device_id, ctx, VISION_STREAM_WIDE_ROAD);

  LOGD("cameras_init: creating PubMaster");
  s->pm = new PubMaster({"roadCameraState", "driverCameraState", "wideRoadCameraState", "thumbnail"});
  LOGD("cameras_init: DONE");
}

void cameras_open(MultiCameraState *s) {
  LOG("-- Opening devices");
  DEBUG_LOG("cameras_open: starting");
  s->wide_road_cam.camera_open(s, 0, !env_disable_wide_road);
  DEBUG_LOG("cameras_open: wide road camera opened enabled=%d", s->wide_road_cam.enabled);
  s->road_cam.camera_open(s, 1, !env_disable_road);
  DEBUG_LOG("cameras_open: road camera opened enabled=%d", s->road_cam.enabled);
  s->driver_cam.camera_open(s, 2, !env_disable_driver);
  DEBUG_LOG("cameras_open: driver camera opened enabled=%d", s->driver_cam.enabled);
}

void CameraState::camera_close() {
  // stop devices
  LOG("-- Stop devices %d", camera_num);

  if (buf.camera_bufs) {
    for (int i = 0; i < FRAME_BUF_COUNT; i++) {
      if (buf.camera_bufs[i].fd >= 0) {
        close(buf.camera_bufs[i].fd);
        buf.camera_bufs[i].fd = -1;
      }
      if (buf.camera_bufs[i].addr != nullptr && buf.camera_bufs[i].addr != MAP_FAILED && buf.camera_bufs[i].mmap_len > 0) {
        munmap(buf.camera_bufs[i].addr, buf.camera_bufs[i].mmap_len);
        buf.camera_bufs[i].addr = nullptr;
        buf.camera_bufs[i].mmap_len = 0;
      }
    }
  }

  // Stop streaming before closing fd to ensure clean buffer release on kernel side
  if (video_fd.fd_ >= 0) {
    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
    if (ioctl(video_fd.fd_, VIDIOC_STREAMOFF, &type) < 0) {
      LOGE("camera %d: VIDIOC_STREAMOFF failed on close errno=%d '%s'", camera_num, errno, strerror(errno));
    } else {
      DEBUG_LOG("camera_close: camera_num=%d VIDIOC_STREAMOFF success", camera_num);
    }
  }

  // unique_fd does not auto-invalidate on manual close; clear fd_ to avoid double-close.
  if (ctrl_fd.fd_ >= 0) {
    close(ctrl_fd.fd_);
    ctrl_fd.fd_ = -1;
  }
  if (video_fd.fd_ >= 0) {
    close(video_fd.fd_);
    video_fd.fd_ = -1;
  }
  if (csiphy_fd.fd_ >= 0) {
    close(csiphy_fd.fd_);
    csiphy_fd.fd_ = -1;
  }

  LOGD("destroyed session %d", camera_num);
}

void cameras_close(MultiCameraState *s) {
  DEBUG_LOG("cameras_close: starting");
  s->driver_cam.camera_close();
  DEBUG_LOG("cameras_close: driver_cam closed");
  s->road_cam.camera_close();
  DEBUG_LOG("cameras_close: road_cam closed");
  s->wide_road_cam.camera_close();
  DEBUG_LOG("cameras_close: wide_road_cam closed");

  DEBUG_LOG("cameras_close: deleting pm");
  delete s->pm;

  // restart rkaiq 3A server
  DEBUG_LOG("cameras_close: killing rkaiq_3A_server");
  system("sudo killall -q /usr/kommu/rkaiq_3A_server || true");
  DEBUG_LOG("cameras_close: sleeping 4s for rkaiq restart");
  usleep(4000000);  // blocks for 4 seconds (was 2.5s)
  DEBUG_LOG("cameras_close: starting rkaiq_3A_server");
  system("sudo /usr/kommu/rkaiq_3A_server &");
  DEBUG_LOG("cameras_close: DONE");
}

static void process_driver_camera(MultiCameraState *s, CameraState *c, uint32_t cnt) {
  MessageBuilder msg;
  auto framed = msg.initEvent().initDriverCameraState();
  fill_frame_data(framed, c->buf.cur_frame_data, c);

  s->pm->send("driverCameraState", msg);
}


static void process_road_camera(MultiCameraState *s, CameraState *c, uint32_t cnt) {
  const CameraBuf *b = &c->buf;

  MessageBuilder msg;
  auto framed = c == &s->road_cam ? msg.initEvent().initRoadCameraState() : msg.initEvent().initWideRoadCameraState();
  fill_frame_data(framed, b->cur_frame_data, c);
  if (env_log_raw_frames && c == &s->road_cam && cnt % 100 == 5) {  // no overlap with qlog decimation
    framed.setImage(get_raw_frame_image(b));
  }
  LOGT(c->buf.cur_frame_data.frame_id, "%s: Image set", c == &s->road_cam ? "RoadCamera" : "WideRoadCamera");

  s->pm->send(c == &s->road_cam ? "roadCameraState" : "wideRoadCameraState", msg);
}


#define THRESHOLD 10000000
bool check_timestamp_sync(uint64_t *t1, int len1, uint64_t *t2, int len2) {
    int i = 0, j = 0;

    while (i < len1 && j < len2) {
        uint64_t diff = t1[i] > t2[j] ? t1[i] - t2[j] : t2[j] - t1[i];
        if (diff <= THRESHOLD) {
            return true;
        }

        // Move the pointer with smaller timestamp forward
        if (t1[i] < t2[j]) {
            i++;
        } else {
            j++;
        }
    }

    return false; // No match within threshold
}

#define SYNC_CHECK_LEN 5
#define SYNC_CHECK_COUNT 40
void cameras_run(MultiCameraState *s) {
  LOG("-- Starting threads");
  DEBUG_LOG("cameras_run: starting threads");
  std::vector<std::thread> threads;
  if (s->driver_cam.enabled) threads.push_back(start_process_thread(s, &s->driver_cam, process_driver_camera));
  if (s->road_cam.enabled) threads.push_back(start_process_thread(s, &s->road_cam, process_road_camera));
  if (s->wide_road_cam.enabled) threads.push_back(start_process_thread(s, &s->wide_road_cam, process_road_camera));
  DEBUG_LOG("cameras_run: threads started, calling stream_start");

  DEBUG_LOG("cameras_run: stream_start road_cam enabled=%d", s->road_cam.enabled);
  s->road_cam.stream_start();
  DEBUG_LOG("cameras_run: stream_start wide_road_cam enabled=%d", s->wide_road_cam.enabled);
  s->wide_road_cam.stream_start();
  DEBUG_LOG("cameras_run: stream_start driver_cam enabled=%d", s->driver_cam.enabled);
  s->driver_cam.stream_start();
  DEBUG_LOG("cameras_run: all streams started");

  uint64_t road_cam_ts[SYNC_CHECK_LEN];
  uint64_t wide_cam_ts[SYNC_CHECK_LEN];
  int count = 0;
  int poll_timeout_count = 0;

  // poll events
  LOG("-- Dequeueing Video events");
  DEBUG_LOG("cameras_run: entering poll loop");
  while (!do_exit) {
    struct pollfd fds[3] = {
      { .fd = s->driver_cam.video_fd, .events = POLLPRI | POLLIN },
      { .fd = s->road_cam.video_fd, .events = POLLPRI | POLLIN },
      { .fd = s->wide_road_cam.video_fd, .events = POLLPRI | POLLIN }
    };

    int ret = poll(fds, std::size(fds), 1000);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      DEBUG_LOG_ERR("cameras_run: poll FAILED ret=%d errno=%d", ret, errno);
      LOGE("poll failed (%d - %d)", ret, errno);
      break;
    }

    if (ret == 0) {
      poll_timeout_count++;
      if (poll_timeout_count % 100 == 0) {
        DEBUG_LOG("cameras_run: poll timeout #%d (no events)", poll_timeout_count);
      }
      if (poll_timeout_count > 300) {
        DEBUG_LOG_ERR("cameras_run: poll timeout for 300 iterations! driver=%d road=%d wide=%d",
                      s->driver_cam.enabled, s->road_cam.enabled, s->wide_road_cam.enabled);
        poll_timeout_count = 0;
      }
      continue;
    }
    poll_timeout_count = 0;

    for (int i = 0; i < 3; i++) {
      if (fds[i].revents & (POLLPRI | POLLIN)) {
        DEBUG_LOG("cameras_run: poll event on camera %d revents=0x%x", i, fds[i].revents);
        // Dequeue buffers for the corresponding camera if the file descriptor is ready
        switch (i) {
          case 0:
            s->driver_cam.dequeue_buf();
            break;
          case 1:
            s->road_cam.dequeue_buf();
            count++;
            if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
              road_cam_ts[count % SYNC_CHECK_COUNT] = s->road_cam.cap_time;
            }
            break;
          case 2:
            s->wide_road_cam.dequeue_buf();
            if (count <= (SYNC_CHECK_COUNT + SYNC_CHECK_LEN - 1) && count >= SYNC_CHECK_COUNT) {
              wide_cam_ts[count % SYNC_CHECK_COUNT] = s->wide_road_cam.cap_time;
            }
            break;
        }
      }
    }

    // Check road and wide camera timestamp sync after initial window is fully populated.
    // We wait until count > SYNC_CHECK_COUNT+SYNC_CHECK_LEN so both circular arrays
    // have valid, paired entries. Then check every SYNC_CHECK_LEN road_cam frames.
    if (count > SYNC_CHECK_COUNT + SYNC_CHECK_LEN && (count - SYNC_CHECK_COUNT) % SYNC_CHECK_LEN == 0) {
      if (!check_timestamp_sync(road_cam_ts, SYNC_CHECK_LEN, wide_cam_ts, SYNC_CHECK_LEN)) {
        DEBUG_LOG_ERR("cameras_run: camera timestamps out of sync at count=%d, road=%lu wide=%lu",
                      count, (unsigned long)road_cam_ts[0],
                      (unsigned long)wide_cam_ts[0]);
      }
    }
  }

  DEBUG_LOG("cameras_run: exiting loop");
  LOG("************** STOPPING **************");

  for (auto &t : threads) t.join();

  // Stop thumbnail worker before cameras_close() destroys PubMaster.
  stop_thumbnail_worker();
  cameras_close(s);
}

