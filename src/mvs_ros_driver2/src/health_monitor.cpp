#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <deque>
#include <sys/statvfs.h>
#include <sys/stat.h>
#include <unistd.h>

class ZenithHealthMonitor : public rclcpp::Node {
public:
  ZenithHealthMonitor() : Node("zenith_health_monitor") {
    start_time_ = std::chrono::steady_clock::now();

    rclcpp::QoS sensor_qos(50);
    sensor_qos.best_effort();

    rclcpp::QoS imu_qos(1000);
    imu_qos.best_effort();

    rclcpp::QoS cam_qos(20);
    cam_qos.reliable();

    sub_cam_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", cam_qos,
        [this](const sensor_msgs::msg::Image::ConstSharedPtr &/*msg*/) {
          auto now = std::chrono::steady_clock::now();
          cam_times_.push_back(now);
          last_cam_time_ = now;
        });

    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/livox/imu", imu_qos,
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr &/*msg*/) {
          auto now = std::chrono::steady_clock::now();
          imu_times_.push_back(now);
          last_imu_time_ = now;
        });

    sub_lidar_ = this->create_generic_subscription(
        "/livox/lidar", "livox_ros_driver2/msg/CustomMsg", sensor_qos,
        [this](std::shared_ptr<rclcpp::SerializedMessage> /*msg*/) {
          auto now = std::chrono::steady_clock::now();
          lidar_times_.push_back(now);
          last_lidar_time_ = now;
        });

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&ZenithHealthMonitor::update_health, this));

    RCLCPP_INFO(this->get_logger(), "C++ Zenith Health Monitor initialized!");
  }

  void update_health() {
    auto now = std::chrono::steady_clock::now();
    auto window_cutoff = now - std::chrono::seconds(2);

    while (!cam_times_.empty() && cam_times_.front() < window_cutoff) cam_times_.pop_front();
    while (!lidar_times_.empty() && lidar_times_.front() < window_cutoff) lidar_times_.pop_front();
    while (!imu_times_.empty() && imu_times_.front() < window_cutoff) imu_times_.pop_front();

    float uptime = std::chrono::duration<float>(now - start_time_).count();
    float window_sec = (uptime < 2.0f) ? std::max(uptime, 0.5f) : 2.0f;

    float cam_fps = std::round((cam_times_.size() / window_sec) * 10.0f) / 10.0f;
    float lidar_fps = std::round((lidar_times_.size() / window_sec) * 10.0f) / 10.0f;
    float imu_fps = std::round((imu_times_.size() / window_sec) * 10.0f) / 10.0f;

    bool is_init = (uptime < 7.0f);

    bool cam_has_data = (last_cam_time_.time_since_epoch().count() > 0);
    bool lidar_has_data = (last_lidar_time_.time_since_epoch().count() > 0);
    bool imu_has_data = (last_imu_time_.time_since_epoch().count() > 0);

    bool cam_ok = is_init ? cam_has_data : (cam_has_data && (now - last_cam_time_ < std::chrono::seconds(3)));
    bool lidar_ok = is_init ? lidar_has_data : (lidar_has_data && (now - last_lidar_time_ < std::chrono::seconds(3)));
    bool imu_ok = is_init ? imu_has_data : (imu_has_data && (now - last_imu_time_ < std::chrono::seconds(2)));

    bool overall_ok = is_init ? true : (cam_ok && lidar_ok && imu_ok);

    std::string alert_level = "ok";
    std::string alert_msg = "";
    if (!is_init) {
      if (!cam_ok) { alert_level = "error"; alert_msg += "📷 카메라 중단 "; }
      if (!lidar_ok) { alert_level = "error"; alert_msg += "📡 라이다 중단 "; }
      if (!imu_ok) { alert_level = "error"; alert_msg += "🧭 IMU 중단 "; }
    }

    // RAM space & health
    float mem_total_gb = 11.3f;
    float mem_avail_gb = 11.3f;
    float mem_used_gb = 0.0f;
    float mem_used_pct = 0.0f;
    std::ifstream mem_file("/proc/meminfo");
    if (mem_file.is_open()) {
      std::string key;
      uint64_t val_kb;
      std::string unit;
      uint64_t total_kb = 0, avail_kb = 0;
      while (mem_file >> key >> val_kb >> unit) {
        if (key == "MemTotal:") total_kb = val_kb;
        else if (key == "MemAvailable:") avail_kb = val_kb;
        if (total_kb > 0 && avail_kb > 0) break;
      }
      if (total_kb > 0) {
        mem_total_gb = std::round((total_kb / (1024.0f * 1024.0f)) * 10.0f) / 10.0f;
        mem_avail_gb = std::round((avail_kb / (1024.0f * 1024.0f)) * 10.0f) / 10.0f;
        mem_used_gb = std::round((mem_total_gb - mem_avail_gb) * 10.0f) / 10.0f;
        mem_used_pct = std::round(((float)(total_kb - avail_kb) / total_kb * 100.0f) * 10.0f) / 10.0f;
      }
    }

    // CPU load & temperature
    float cpu_usage_pct = 0.0f;
    int cpu_cores = sysconf(_SC_NPROCESSORS_ONLN);
    std::ifstream stat_file("/proc/stat");
    if (stat_file.is_open()) {
      std::string cpu_label;
      uint64_t user, nice, sys, idle, iowait, irq, softirq, steal;
      if (stat_file >> cpu_label >> user >> nice >> sys >> idle >> iowait >> irq >> softirq >> steal) {
        uint64_t idle_all = idle + iowait;
        uint64_t total_all = user + nice + sys + idle_all + irq + softirq + steal;
        if (prev_cpu_total_ > 0 && total_all > prev_cpu_total_) {
          uint64_t diff_idle = idle_all - prev_cpu_idle_;
          uint64_t diff_total = total_all - prev_cpu_total_;
          cpu_usage_pct = std::round(((float)(diff_total - diff_idle) / diff_total * 100.0f) * 10.0f) / 10.0f;
        }
        prev_cpu_idle_ = idle_all;
        prev_cpu_total_ = total_all;
      }
    }

    float cpu_temp_c = 0.0f;
    std::ifstream temp_file("/sys/class/thermal/thermal_zone0/temp");
    if (temp_file.is_open()) {
      int temp_milli = 0;
      if (temp_file >> temp_milli) {
        cpu_temp_c = std::round((temp_milli / 1000.0f) * 10.0f) / 10.0f;
      }
    }

    // Evaluate RAM & CPU alert
    bool mem_ok = (mem_avail_gb >= 1.0f);
    bool cpu_ok = (cpu_temp_c < 85.0f);
    if (!is_init) {
      if (mem_avail_gb < 0.5f) {
        alert_level = "error";
        alert_msg += "🚨 RAM 고갈 임박 (" + std::to_string((int)(mem_avail_gb * 1024)) + "MB) OOM 방지 긴급 정상 저장 트리거! ";
        RCLCPP_FATAL(this->get_logger(), "CRITICAL: Available RAM < 500MB! Triggering emergency graceful stop to prevent Kernel OOM crash!");
        static bool emergency_triggered = false;
        if (!emergency_triggered) {
          emergency_triggered = true;
          int ret = system("pkill -2 -f '[f]astlivo_mapping'; sleep 1; pkill -2 -f '[r]os2 bag record'");
          (void)ret;
        }
      } else if (mem_avail_gb < 0.8f) {
        alert_level = "error";
        alert_msg += "🧠 RAM 위험 (" + std::to_string((int)(mem_avail_gb * 1024)) + "MB 남음) ";
      } else if (mem_avail_gb < 1.8f && alert_level != "error") {
        alert_level = "warning";
        alert_msg += "🧠 RAM 주의 (" + std::to_string(mem_avail_gb).substr(0, 3) + "GB 남음) ";
      }

      if (mem_avail_gb < 2.2f) {
        static float last_cache_drop = 0.0f;
        if (uptime - last_cache_drop > 15.0f) {
          last_cache_drop = uptime;
          int ret = system("sync; echo 1 > /proc/sys/vm/drop_caches 2>/dev/null &");
          (void)ret;
        }
      }

      if (cpu_temp_c >= 85.0f) {
        alert_level = "error";
        alert_msg += "🔥 CPU 과열 (" + std::to_string((int)cpu_temp_c) + "°C) ";
      } else if (cpu_temp_c >= 78.0f && alert_level != "error") {
        alert_level = "warning";
        alert_msg += "🔥 CPU 고온 (" + std::to_string((int)cpu_temp_c) + "°C) ";
      }
    }

    // Disk space
    struct statvfs stat;
    float disk_free_gb = 0.0f;
    if (statvfs("/root/zenith_ws", &stat) == 0) {
      disk_free_gb = (float)(stat.f_bavail * stat.f_frsize) / (1024.0f * 1024.0f * 1024.0f);
    }

    double wall_now = std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();

    std::string cam_status = cam_has_data ? (cam_ok ? "정상" : "중단") : (is_init ? "연결 중" : "미연결");
    std::string lidar_status = lidar_has_data ? (lidar_ok ? "정상" : "중단") : (is_init ? "연결 중" : "미연결");
    std::string imu_status = imu_has_data ? (imu_ok ? "정상" : "중단") : (is_init ? "연결 중" : "미연결");
    std::string mem_status = (mem_avail_gb < 0.8f) ? "위험" : ((mem_avail_gb < 1.8f) ? "주의" : "정상");
    std::string cpu_status = (cpu_temp_c >= 85.0f) ? "과열" : ((cpu_temp_c >= 78.0f) ? "주의" : "정상");

    std::ofstream out("/root/zenith_ws/.sensor_status.tmp");
    if (out.is_open()) {
      out << "{\n";
      out << "  \"updated_at\": " << std::fixed << std::setprecision(2) << wall_now << ",\n";
      out << "  \"uptime\": " << uptime << ",\n";
      out << "  \"state\": \"" << (is_init ? "INITIALIZING" : "ACTIVE") << "\",\n";
      out << "  \"overall_ok\": " << (overall_ok ? "true" : "false") << ",\n";
      out << "  \"alert_level\": \"" << alert_level << "\",\n";
      out << "  \"alert_msg\": " << (alert_msg.empty() ? "null" : ("\"" + alert_msg + "\"")) << ",\n";
      out << "  \"camera\": { \"ok\": " << (cam_ok ? "true" : "false") << ", \"fps\": " << cam_fps << ", \"status\": \"" << cam_status << "\" },\n";
      out << "  \"lidar\": { \"ok\": " << (lidar_ok ? "true" : "false") << ", \"fps\": " << lidar_fps << ", \"status\": \"" << lidar_status << "\" },\n";
      out << "  \"imu\": { \"ok\": " << (imu_ok ? "true" : "false") << ", \"fps\": " << imu_fps << ", \"status\": \"" << imu_status << "\" },\n";
      out << "  \"memory\": { \"ok\": " << (mem_ok ? "true" : "false") << ", \"total_gb\": " << mem_total_gb << ", \"used_gb\": " << mem_used_gb << ", \"avail_gb\": " << mem_avail_gb << ", \"percent\": " << mem_used_pct << ", \"status\": \"" << mem_status << "\" },\n";
      out << "  \"cpu\": { \"ok\": " << (cpu_ok ? "true" : "false") << ", \"percent\": " << cpu_usage_pct << ", \"temp_c\": " << cpu_temp_c << ", \"cores\": " << cpu_cores << ", \"status\": \"" << cpu_status << "\" },\n";
      out << "  \"bag\": { \"ok\": true, \"size_mb\": 0, \"status\": \"기록 중\" },\n";
      out << "  \"disk_free_gb\": " << disk_free_gb << "\n";
      out << "}\n";
      out.close();
      rename("/root/zenith_ws/.sensor_status.tmp", "/root/zenith_ws/.sensor_status.json");
      chmod("/root/zenith_ws/.sensor_status.json", 0666);
    }
  }

private:
  std::chrono::steady_clock::time_point start_time_;
  std::chrono::steady_clock::time_point last_cam_time_{};
  std::chrono::steady_clock::time_point last_lidar_time_{};
  std::chrono::steady_clock::time_point last_imu_time_{};

  uint64_t prev_cpu_idle_ = 0;
  uint64_t prev_cpu_total_ = 0;

  std::deque<std::chrono::steady_clock::time_point> cam_times_;
  std::deque<std::chrono::steady_clock::time_point> lidar_times_;
  std::deque<std::chrono::steady_clock::time_point> imu_times_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::GenericSubscription::SharedPtr sub_lidar_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ZenithHealthMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
