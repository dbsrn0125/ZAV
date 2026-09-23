#include "MvCameraControl.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <rclcpp/rclcpp.hpp>
#include <signal.h>
#include <stdio.h>
#include <sys/ipc.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <unistd.h>

// 日志输出 port to ROS2
#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_ERROR(...)                                                         \
  RCLCPP_ERROR(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)
#define ROS_DEBUG(...)                                                         \
  RCLCPP_DEBUG(rclcpp::get_logger("mvs_driver"), __VA_ARGS__)

using namespace std;

class MvsSdkSession {
public:
  MvsSdkSession() : status_(MV_CC_Initialize()) {}

  ~MvsSdkSession() {
    if (status_ == MV_OK) {
      MV_CC_Finalize();
    }
  }

  int status() const { return status_; }

  MvsSdkSession(const MvsSdkSession &) = delete;
  MvsSdkSession &operator=(const MvsSdkSession &) = delete;

private:
  int status_;
};

struct time_stamp {
  int64_t high;
  int64_t low;
};
time_stamp *pointt = nullptr;

enum PixelFormat : unsigned int {
  RGB8 = 0x02180014,
  BayerRG8 = 0x01080009,
  BayerRG12Packed = 0x010C002B,
  BayerGB12Packed = 0x010C002C,
  BayerGB8 = 0x0108000A
};

// unsigned int g_nPayloadSize = 0;
bool is_undistorted = true;
std::atomic<bool> exit_flag(false);
std::atomic<uint64_t> last_frame_time_ms(0);
std::atomic<bool> has_received_first_frame(false);
int width, height;
rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub;
std::mutex g_queue_mutex;
std::condition_variable g_queue_cv;
std::deque<sensor_msgs::msg::Image> g_image_queue;
const size_t MAX_QUEUE_SIZE = 5;
std::vector<PixelFormat> PIXEL_FORMAT = {RGB8, BayerRG8, BayerRG12Packed,
                                         BayerGB12Packed, BayerGB8};
std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
float image_scale = 0.0;
int trigger_enable = 1;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo) {
  if (NULL == pstMVDevInfo) {
    ROS_ERROR("The Pointer of pstMVDevInfo is NULL!");
    return false;
  }
  if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE) {
    int nIp1 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
    int nIp2 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
    int nIp3 =
        ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
    int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);

    ROS_INFO("[GigE] device");
    ROS_INFO("Device Model Name: %s",
             pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
    ROS_INFO("CurrentIp: %d.%d.%d.%d", nIp1, nIp2, nIp3, nIp4);
    ROS_INFO("SerialNumber: %s",
             pstMVDevInfo->SpecialInfo.stGigEInfo.chSerialNumber);
  } else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE) {
    ROS_INFO("[USB] device");
    ROS_INFO("Device Model Name: %s",
             pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
    ROS_INFO("SerialNumber: %s",
             pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
  } else {
    ROS_WARN("Not support.");
  }
  return true;
}

void setParams(void *handle, const std::string &params_file) {
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR(msg.c_str());
    exit(-1);
  }
  image_scale = Params["image_scale"];
  if (image_scale < 0.1)
    image_scale = 1;
  int ExposureTimeLower = Params["AutoExposureTimeLower"];
  int ExposureTimeUpper = Params["AutoExposureTimeUpper"];
  int ExposureTime = Params["ExposureTime"];
  int ExposureAutoMode = Params["ExposureAutoMode"];
  int GainAuto = Params["GainAuto"];
  float Gain = Params["Gain"];
  float Gamma = Params["Gamma"];
  int GammaSlector = Params["GammaSelector"];
  int nRet;

  // 设置曝光模式
  nRet = MV_CC_SetExposureAutoMode(handle, ExposureAutoMode);
  std::string msg =
      "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];

  if (MV_OK == nRet) {
    ROS_INFO(msg.c_str());
  } else {
    if (ExposureAutoMode == 2) {
      ROS_WARN("Fail to set Exposure Auto Mode to Continues");
    } else {
      ROS_INFO(msg.c_str());
    }
  }

  // 如果是自动曝光
  if (ExposureAutoMode == 2) {
    nRet = MV_CC_SetAutoExposureTimeLower(handle, ExposureTimeLower);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Lower: " + std::to_string(ExposureTimeLower) +
          "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time Lower");
    }
    nRet = MV_CC_SetAutoExposureTimeUpper(handle, ExposureTimeUpper);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time Upper: " + std::to_string(ExposureTimeUpper) +
          "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time Upper");
    }
  }

  // 如果是固定曝光
  if (ExposureAutoMode == 0) {
    nRet = MV_CC_SetExposureTime(handle, ExposureTime);
    if (MV_OK == nRet) {
      std::string msg =
          "Set Exposure Time: " + std::to_string(ExposureTime) + "us";
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure Time");
    }
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);

  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gain auto mode");
  }

  if (GainAuto == 0) {
    nRet = MV_CC_SetGain(handle, Gain);
    if (MV_OK == nRet) {
      std::string msg = "Set Gain: " + std::to_string(Gain);
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Gain");
    }
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set GammaSlector");
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gamma");
  }

  float FrameRate = 10.0f;
  if (!Params["FrameRate"].empty()) {
    FrameRate = (float)Params["FrameRate"];
  }

  if (FrameRate > 0.0f) {
    nRet = MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", true);
    if (MV_OK == nRet) {
      nRet = MV_CC_SetFloatValue(handle, "AcquisitionFrameRate", FrameRate);
      if (MV_OK == nRet) {
        ROS_INFO("Set AcquisitionFrameRate: %.1f Hz", FrameRate);
      } else {
        ROS_WARN("Fail to set AcquisitionFrameRate to %.1f Hz, nRet [0x%x]", FrameRate, nRet);
      }
    } else {
      ROS_WARN("Fail to enable AcquisitionFrameRateEnable, nRet [0x%x]", nRet);
    }
  } else {
    MV_CC_SetBoolValue(handle, "AcquisitionFrameRateEnable", false);
  }

  // Hardware Binning configuration (2x2 Hardware Binning delivers native 640x512 over USB)
  int binning = 2; // Default to 2x2 for MV-CU013 (75% USB traffic reduction)
  if (!Params["Binning"].empty()) {
    binning = Params["Binning"];
  } else if (image_scale >= 0.99f) {
    binning = 1;
  }

  if (binning > 1) {
    nRet = MV_CC_SetEnumValue(handle, "BinningHorizontal", binning);
    int nRetV = MV_CC_SetEnumValue(handle, "BinningVertical", binning);
    if (MV_OK == nRet && MV_OK == nRetV) {
      ROS_INFO("Set Hardware Binning to %dx%d (Native 640x512 over USB, 75%% bandwidth reduction)", binning, binning);
    } else {
      ROS_WARN("Failed to set Hardware Binning %dx%d (ret: 0x%x, 0x%x)", binning, binning, nRet, nRetV);
    }
  } else {
    MV_CC_SetEnumValue(handle, "BinningHorizontal", 1);
    MV_CC_SetEnumValue(handle, "BinningVertical", 1);
    ROS_INFO("Hardware Binning disabled (Native full resolution)");
  }
}

void SignalHandler(int signal) {
  if (signal == SIGINT || signal == SIGTERM) {
    fprintf(stderr, "\nReceived shutdown signal (%d), exiting...\n", signal);
    exit_flag.store(true);
    g_queue_cv.notify_all();
  }
}

void SetupSignalHandler() {
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = SignalHandler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);
  sigaction(SIGTERM, &sigIntHandler, NULL);
}

static void *GrabThread(void *pUser) {
  int nRet = MV_OK;

  std::vector<unsigned char> rgb_buffer;
  unsigned char *pDataForRGB = NULL;
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  nRet = MV_CC_GetIntValue(pUser, "PayloadSize", &stParam);
  if (MV_OK != nRet) {
    ROS_ERROR("Get PayloadSize fail! nRet [0x%x]", nRet);
    return NULL;
  }
  ROS_INFO("Get PayloadSize success! val [%d]", stParam.nCurValue);

  MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
  MV_FRAME_OUT stImageInfo = {0};

  int consecutive_timeouts = 0;
  ROS_INFO("GrabThread: Capture loop started.");
  while (!exit_flag.load() && rclcpp::ok()) {

    nRet = MV_CC_GetImageBuffer(pUser, &stImageInfo, 1000);

    if (nRet == MV_OK) {
      consecutive_timeouts = 0;
      last_frame_time_ms.store(
          std::chrono::duration_cast<std::chrono::milliseconds>(
              std::chrono::steady_clock::now().time_since_epoch())
              .count());
      has_received_first_frame.store(true);

      rclcpp::Time rcv_time;
      if (trigger_enable && pointt != nullptr && pointt != MAP_FAILED && pointt->low != 0) {
        int64_t b = pointt->low;
        double time_pc = b / 1000000000.0;
        rcv_time = rclcpp::Time(static_cast<int64_t>(time_pc * 1e9));
      } else {
        rcv_time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      }

      size_t needed_size = (size_t)stImageInfo.stFrameInfo.nExtendWidth *
                           stImageInfo.stFrameInfo.nExtendHeight * 4 + 2048;
      if (rgb_buffer.size() < needed_size) {
        rgb_buffer.resize(needed_size);
      }
      pDataForRGB = rgb_buffer.data();

      stConvertParam.nWidth = stImageInfo.stFrameInfo.nExtendWidth;
      stConvertParam.nHeight = stImageInfo.stFrameInfo.nExtendHeight;
      stConvertParam.pSrcData = stImageInfo.pBufAddr;
      stConvertParam.nSrcDataLen = stImageInfo.stFrameInfo.nFrameLenEx;
      stConvertParam.enSrcPixelType = stImageInfo.stFrameInfo.enPixelType;
      stConvertParam.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      stConvertParam.pDstBuffer = pDataForRGB;
      stConvertParam.nDstBufferSize = rgb_buffer.size();

      nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);

      if (MV_OK != nRet) {
        ROS_WARN("MV_CC_ConvertPixelType failed! nRet [%x], skipping frame", nRet);
        MV_CC_FreeImageBuffer(pUser, &stImageInfo);
        continue;
      }

      int frame_w = stImageInfo.stFrameInfo.nWidth;
      int frame_h = stImageInfo.stFrameInfo.nHeight;
      int frame_num = stImageInfo.stFrameInfo.nFrameNum;

      // Free hardware SDK buffer IMMEDIATELY after convert!
      MV_CC_FreeImageBuffer(pUser, &stImageInfo);

      cv::Mat srcImage(frame_h, frame_w, CV_8UC3, pDataForRGB);

      if (image_scale > 0.0 && image_scale < 0.99f && (srcImage.cols > 640 || srcImage.rows > 512)) {
        cv::resize(srcImage, srcImage,
                   cv::Size(srcImage.cols * image_scale, srcImage.rows * image_scale),
                   cv::INTER_LINEAR);
      }

      sensor_msgs::msg::Image msg;
      msg.header.stamp = rcv_time;
      msg.height = srcImage.rows;
      msg.width = srcImage.cols;
      msg.encoding = "rgb8";
      msg.is_bigendian = false;
      msg.step = srcImage.step;
      msg.data.assign(srcImage.data,
                      srcImage.data + srcImage.total() * srcImage.elemSize());

      // Push into decoupled publish queue (never blocks GrabThread!)
      {
        std::lock_guard<std::mutex> lock(g_queue_mutex);
        if (g_image_queue.size() >= MAX_QUEUE_SIZE) {
          g_image_queue.pop_front();
        }
        g_image_queue.push_back(std::move(msg));
      }
      g_queue_cv.notify_one();

      ROS_DEBUG("GrabThread: frame #%d queued", frame_num);
    } else {
      consecutive_timeouts++;
      ROS_WARN("Capture timeout (%d consecutive timeouts, nRet [0x%x]), retrying...",
               consecutive_timeouts, nRet);
      if (consecutive_timeouts == 3) {
        ROS_WARN("Camera stream appears stalled (3s). Resetting grab stream...");
        MV_CC_StopGrabbing(pUser);
        usleep(100000);
        int resetRet = MV_CC_StartGrabbing(pUser);
        if (resetRet == MV_OK) {
          ROS_INFO("Camera grab stream restarted successfully.");
          consecutive_timeouts = 0;
        } else {
          ROS_ERROR("Failed to restart camera grab stream! nRet [0x%x]", resetRet);
        }
      } else if (consecutive_timeouts >= 6) {
        ROS_ERROR("Camera stream dead for %d consecutive seconds. Exiting node...",
                  consecutive_timeouts);
        exit_flag.store(true);
        g_queue_cv.notify_all();
        break;
      }
    }
  }

  g_queue_cv.notify_all();
  ROS_INFO("GrabThread exited cleanly.");
  return NULL;
}

static void *PublishThread(void *pUser) {
  (void)pUser;
  ROS_INFO("PublishThread: started.");
  while (!exit_flag.load() && rclcpp::ok()) {
    sensor_msgs::msg::Image msg;
    {
      std::unique_lock<std::mutex> lock(g_queue_mutex);
      g_queue_cv.wait_for(lock, std::chrono::milliseconds(100), [] {
        return !g_image_queue.empty() || exit_flag.load();
      });

      if (exit_flag.load() || !rclcpp::ok()) {
        break;
      }
      if (g_image_queue.empty()) {
        continue;
      }

      msg = std::move(g_image_queue.front());
      g_image_queue.pop_front();
    }

    if (pub) {
      pub->publish(std::move(msg));
    }
  }
  ROS_INFO("PublishThread exited cleanly.");
  return NULL;
}

int main(int argc, char **argv) {

  rclcpp::init(argc, argv);
  if (argc < 2 || argv[1] == nullptr) {
    ROS_ERROR("Usage: mvs_camera_node <settings_file>");
    rclcpp::shutdown();
    return -1;
  }

  std::string params_file = std::string(argv[1]);
  // cv::FileStorage Params(params_file, cv::FileStorage::READ);
  // trigger_enable = Params["TriggerEnable"];
  // std::string expect_serial_number = Params["SerialNumber"];
  // std::string pub_topic = Params["TopicName"];
  // int PixelFormat = Params["PixelFormat"];

  MvsSdkSession sdk_session;
  int nRet = sdk_session.status();
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_Initialize fail! nRet [%x]", nRet);
    return -1;
  }

  void *handle = NULL;
  rclcpp::Rate loop_rate(10);
  cv::FileStorage Params(params_file, cv::FileStorage::READ);
  if (!Params.isOpened()) {
    string msg = "Failed to open settings file at:" + params_file;
    ROS_ERROR(msg.c_str());
    exit(-1);
  }
  ROS_INFO("Load settings from file: %s", params_file.c_str());
  trigger_enable = Params["TriggerEnable"];
  std::string expect_serial_number = Params["SerialNumber"];
  std::string pub_topic = Params["TopicName"];
  int PixelFormat = Params["PixelFormat"];

  auto node = rclcpp::Node::make_shared("mvs_trigger");
  rclcpp::QoS cam_pub_qos(10);
  cam_pub_qos.reliable();
  pub = node->create_publisher<sensor_msgs::msg::Image>(pub_topic, cam_pub_qos);

  if (trigger_enable) {
    const char *home_dir = std::getenv("HOME");
    pointt = nullptr;
    if (home_dir != nullptr && home_dir[0] != '\0') {
      std::string path_for_time_stamp = std::string(home_dir) + "/timeshare";
      int fd = open(path_for_time_stamp.c_str(), O_RDWR);
      if (fd != -1) {
        pointt = (time_stamp *)mmap(NULL, sizeof(time_stamp), PROT_READ | PROT_WRITE,
                                    MAP_SHARED, fd, 0);
        if (pointt == MAP_FAILED) {
          pointt = nullptr;
        }
        close(fd);
      }
    }
    if (pointt == nullptr) {
      ROS_INFO("No external timeshare file found. Using ROS system clock for hardware triggered timestamps.");
    } else {
      ROS_INFO("Successfully mapped external timeshare file for hardware timestamps.");
    }
  }

  SetupSignalHandler();

  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));

  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_EnumDevices fail! nRet [%x]", nRet);
    return -1;
  }

  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      ROS_INFO("[device %d]:", i);
      MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == NULL) {
        ROS_ERROR("Device info is NULL for device %d", i);
        return -1;
      }
      PrintDeviceInfo(pDeviceInfo);
    }
  } else {
    ROS_ERROR("Find No Devices!");
    return -1;
  }

  bool find_expect_camera = false;
  unsigned int nIndex = 0;

  if (stDeviceList.nDeviceNum > 1) {
    if (expect_serial_number.empty()) {
      ROS_ERROR("Expected serial number is empty!");
      return -1;
    }
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      if (stDeviceList.pDeviceInfo[i] == NULL) {
        ROS_ERROR("Device info is NULL for device %d", i);
        continue;
      }

      std::string serial_number;
      if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_USB_DEVICE) {
        serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stUsb3VInfo.chSerialNumber);
      } else if (stDeviceList.pDeviceInfo[i]->nTLayerType == MV_GIGE_DEVICE) {
        serial_number =
            std::string((char *)stDeviceList.pDeviceInfo[i]
                            ->SpecialInfo.stGigEInfo.chSerialNumber);
      } else {
        ROS_ERROR("Unknown device type!");
        continue;
      }
      if (serial_number.empty()) {
        ROS_ERROR("Serial number is empty for device %d", i);
        continue;
      }
      if (expect_serial_number == serial_number) {
        find_expect_camera = true;
        nIndex = i;
        break;
      }
    }
    if (!find_expect_camera) {
      std::string msg =
          "Can not find the camera with serial number " + expect_serial_number;
      ROS_ERROR(msg.c_str());
      return -1;
    }
  } else {
    nIndex = 0;
  }

  // select device and create handle
  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CreateHandle fail! nRet [%x]", nRet);
    return -1;
  }

  // open device with retries (in case USB is settling from previous session)
  int open_retries = 0;
  while (open_retries < 5) {
    nRet = MV_CC_OpenDevice(handle);
    if (MV_OK == nRet) {
      break;
    }
    open_retries++;
    ROS_WARN("MV_CC_OpenDevice attempt %d failed (nRet [0x%x]), retrying in 500ms...", open_retries, nRet);
    usleep(500000);
  }
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_OpenDevice failed after retries! nRet [%x]", nRet);
    MV_CC_DestroyHandle(handle);
    return -1;
  }

  // Frame rate will be configured in setParams() based on yaml config

  // MVCC_INTVALUE stParam;
  // memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  // nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
  // if (MV_OK != nRet) {
  //   printf("Get PayloadSize fail\n");
  //   return -1;
  // }
  // g_nPayloadSize = stParam.nCurValue * 3;

  nRet = MV_CC_SetEnumValue(
      handle, "PixelFormat",
      PIXEL_FORMAT[PixelFormat]); // BayerRG8 0x01080009 RGB8 0x02180014
                                  // BayerRG12Packed 0x010C002B
  if (nRet != MV_OK) {
    ROS_ERROR("Pixel setting can't work.");
    return -1;
  }

  setParams(handle, params_file);

  // set trigger mode as on
  nRet = MV_CC_SetEnumValue(handle, "TriggerMode", trigger_enable);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_SetTriggerMode fail! nRet [%x]", nRet);
    return -1;
  } else {
    ROS_INFO("Set TriggerMode [%s]", trigger_enable == 0 ? "OFF" : "ON");
  }

  // set trigger source
  nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_SetTriggerSource fail! nRet [%x]", nRet);
    return -1;
  }

  // Set Grab Strategy to OneByOne for sequential, lossless frame capture
  nRet = MV_CC_SetGrabStrategy(handle, MV_GrabStrategy_OneByOne);
  if (MV_OK != nRet) {
    ROS_WARN("MV_CC_SetGrabStrategy fail! nRet [0x%x]", nRet);
  } else {
    ROS_INFO("Set GrabStrategy to MV_GrabStrategy_OneByOne success.");
  }

  // Set SDK internal image buffer nodes to 20 for ample buffering headroom
  nRet = MV_CC_SetImageNodeNum(handle, 20);
  if (MV_OK != nRet) {
    ROS_WARN("MV_CC_SetImageNodeNum fail! nRet [0x%x]", nRet);
  } else {
    ROS_INFO("Set ImageNodeNum to 20 success.");
  }

  ROS_INFO("Finish all params set! Start grabbing...");
  nRet = MV_CC_StartGrabbing(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("Start Grabbing fail.");
    return -1;
  }
  ROS_INFO("Start Grabbing Success.");

  pthread_t nGrabThreadID;
  nRet = pthread_create(&nGrabThreadID, NULL, GrabThread, handle);
  if (nRet != 0) {
    ROS_ERROR("GrabThread create failed. ret = %d", nRet);
    return -1;
  }
  ROS_INFO("Start GrabThread Success, pid %ld", nGrabThreadID);

  pthread_t nPubThreadID;
  nRet = pthread_create(&nPubThreadID, NULL, PublishThread, NULL);
  if (nRet != 0) {
    ROS_ERROR("PublishThread create failed. ret = %d", nRet);
    return -1;
  }
  ROS_INFO("Start PublishThread Success, pid %ld", nPubThreadID);

  while (!exit_flag.load() && rclcpp::ok()) {
    rclcpp::spin_some(node);
    usleep(100000);

    // Watchdog check: if frames were flowing but stopped for > 10s
    if (has_received_first_frame.load()) {
      uint64_t now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      if (now_ms > last_frame_time_ms.load() + 10000) {
        ROS_ERROR("Watchdog: Camera stream frozen for >10 seconds! Initiating clean device release for ROS 2 auto-respawn...");
        exit_flag.store(true);
        g_queue_cv.notify_all();
        break;
      }
    }
  }

  exit_flag.store(true);
  g_queue_cv.notify_all();

  if (nGrabThreadID) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    pthread_timedjoin_np(nGrabThreadID, NULL, &ts);
    ROS_INFO("GrabThread joined.");
  }

  if (nPubThreadID) {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    ts.tv_sec += 2;
    pthread_timedjoin_np(nPubThreadID, NULL, &ts);
    ROS_INFO("PublishThread joined.");
  }

  nRet = MV_CC_StopGrabbing(handle);
  if (MV_OK != nRet) {
    ROS_WARN("MV_CC_StopGrabbing result: [0x%x]", nRet);
  } else {
    ROS_INFO("MV_CC_StopGrabbing success!");
  }

  nRet = MV_CC_CloseDevice(handle);
  if (MV_OK != nRet) {
    ROS_WARN("MV_CC_CloseDevice result: [0x%x]", nRet);
  } else {
    ROS_INFO("MV_CC_CloseDevice success!");
  }

  nRet = MV_CC_DestroyHandle(handle);
  if (MV_OK != nRet) {
    ROS_WARN("MV_CC_DestroyHandle result: [0x%x]", nRet);
  } else {
    ROS_INFO("MV_CC_DestroyHandle success!");
  }

  if (pointt != nullptr && pointt != MAP_FAILED) {
    munmap(pointt, sizeof(time_stamp));
  }

  if (rclcpp::ok()) {
    ROS_WARN("mvs_camera_node exited unexpectedly. Terminating for respawn.");
    rclcpp::shutdown();
    return 1;
  }

  return 0;
}
