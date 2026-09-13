#include "TelePandaTDPA2010AsWhole.h"

#define MAXLINE 1024

namespace fs = std::filesystem;

// panda_link8 -> panda_hand_tcp. Together with the 0.107 m link7 -> link8
// offset in Kinematics.h, this matches the panda_hand_tcp frame used by the
// maze dataset and deployment URDF.
constexpr double kPandaHandTcpOffsetM = 0.1034;

static inline std::string format_episode_dir(int episode_idx)
{
  char buf[32];
  std::snprintf(buf, sizeof(buf), "episode_%03d", episode_idx);
  return std::string(buf);
}

std::atomic<bool> g_record_active{false};
std::atomic<bool> g_record_started{false};
std::atomic<int64_t> g_record_start_time_ns{0};
std::atomic<bool> g_stop_requested{false};
std::atomic<double> g_follower_gripper_width{0.0};
std::atomic<int> g_cameras_ready{0};
std::atomic<bool> g_camera_capture_failed{false};
std::atomic<bool> g_camera_episode_running{false};
std::atomic<int> g_episode_phase{0};
std::atomic<double> g_lift_target_z{std::numeric_limits<double>::quiet_NaN()};
std::array<std::atomic<uint64_t>, 3> g_camera_committed_frames = {
    std::atomic<uint64_t>{0}, std::atomic<uint64_t>{0},
    std::atomic<uint64_t>{0}};
volatile std::sig_atomic_t g_signal_stop_requested = 0;

enum EpisodePhase
{
  kResetOrAlign = 0,
  kCloseDetected = 1,
  kLifting = 2,
  kLiftReady = 3,
  kRecording = 4,
};

std::mutex g_record_mutex;
std::unique_ptr<Recorder> g_episode_recorder;
std::array<std::thread, 3> g_camera_threads;
fs::path g_recording_base_dir;
fs::path g_current_episode_dir;
std::string g_recording_trial_name;
int g_next_episode_index = 1;
bool g_record_data_enabled = false;
bool g_record_camera_enabled = false;
std::array<std::string, 3> g_recording_camera_serials;
std::array<bool, 3> g_recording_camera_enabled = {true, true, true};
int g_recording_enabled_camera_count = 3;
double g_automatic_lift_m = 0.02;
constexpr int kRobotRecordColumns = 30;
constexpr double kInitialRecordSeconds = 30.0;
constexpr double kRobotRecordSampleSeconds = 0.001;

extern "C" void request_graceful_stop(int)
{
  g_signal_stop_requested = 1;
}

static inline int64_t steady_time_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
             std::chrono::steady_clock::now().time_since_epoch())
      .count();
}

int main(int argc, char **argv)
{
  if (argc != 5)
  {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>"
              << " "
              << "<l or f> <trial_name> <episode_idx>\n"
              << std::endl;
    return -1;
  }
  std::string robot_ip = argv[1];
  std::string trial_name = argv[3];
  int episode_idx = std::stoi(argv[4]);

  std::string leadorfollow = argv[2];

  if (leadorfollow != "l" && leadorfollow != "f")
  {
    std::cerr << "use l or f" << std::endl;
    return -1;
  }
  std::signal(SIGINT, request_graceful_stop);
  std::signal(SIGTERM, request_graceful_stop);

  // ***************** poco *******************
  bool vis = false;
  std::string m_msg;
  nlohmann::json m_json;
  Poco::Net::SocketAddress sa("localhost", 6666);
  Poco::Net::DatagramSocket m_dgs;

  if (vis)
  {
    try
    {
      m_dgs.connect(sa);
      m_json["F_ext"] = {};
      m_json["Q"] = {};
      m_msg = m_json.dump();
    }
    catch (std::exception &e)
    {
      vis = 0;
      std::cout << "[poco] no server" << e.what() << std::endl;
    }
  }

  // ***************** poco *******************

  // =================== data directory per trial ===================
  // Store recordings alongside the project instead of relying on a specific
  // deployment user or a hard-coded /home path. The executable lives in the
  // project's build directory, so its parent directory is the project root.
  fs::path project_dir =
      fs::canonical("/proc/self/exe").parent_path().parent_path();
  fs::path base_dir = project_dir / "data";
  fs::path trial_dir = base_dir / trial_name / format_episode_dir(episode_idx);

  std::cout << "[TelePanda] first episode path = " << trial_dir << std::endl;
  // =================== data directory per trial ===================

  /* Read and parse JSON file parameters */
  // json parameter;
  const fs::path parameter_path =
      project_dir / "src" /
      (leadorfollow == "l" ? "leader_config.json" : "follower_config.json");
  std::ifstream parameter_file(parameter_path);
  if (!parameter_file)
  {
    std::cerr << "failed to open configuration file: " << parameter_path
              << std::endl;
    return -1;
  }
  json parameter;
  try
  {
    parameter_file >> parameter;
  }
  catch (const json::parse_error &error)
  {
    std::cerr << "failed to parse configuration file " << parameter_path
              << ": " << error.what() << std::endl;
    return -1;
  }

  std::string IP_remote_st = parameter["remote_ip"];
  IP_remote = IP_remote_st.c_str();

  bool TDPA_active = parameter["TDPA_active"];
  bool tau_ext_feedback = parameter["tau_ext_feedback"];
  bool record_data = parameter.value("record_data", false);
  bool record_camera = parameter.value("record_camera", false);
  std::array<std::string, 3> camera_serials = {
      "233722072293", "233622071984", "233522077069"};
  std::array<bool, 3> camera_enabled = {true, true, true};
  if (parameter.contains("camera_serials"))
  {
    const auto &configured_serials = parameter.at("camera_serials");
    camera_serials = {
        configured_serials.at("cam1").get<std::string>(),
        configured_serials.at("cam2").get<std::string>(),
        configured_serials.at("cam3").get<std::string>()};
  }
  if (parameter.contains("camera_enabled"))
  {
    const auto &configured_cameras = parameter.at("camera_enabled");
    camera_enabled = {
        configured_cameras.value("cam1", true),
        configured_cameras.value("cam2", true),
        configured_cameras.value("cam3", true)};
  }
  const int enabled_camera_count = static_cast<int>(
      std::count(camera_enabled.begin(), camera_enabled.end(), true));
  if (record_camera && enabled_camera_count == 0)
  {
    std::cerr << "record_camera=true requires at least one enabled camera"
              << std::endl;
    return -1;
  }
  if (camera_serials[0] == camera_serials[1] ||
      camera_serials[0] == camera_serials[2] ||
      camera_serials[1] == camera_serials[2])
  {
    std::cerr << "camera_serials must contain three distinct serial numbers" << std::endl;
    return -1;
  }
  double gripper_grasp_force = parameter.value("gripper_grasp_force", 50.0);
  if (gripper_grasp_force <= 0.0 || gripper_grasp_force > 70.0)
  {
    std::cerr << "gripper_grasp_force must be in the range (0, 70] N" << std::endl;
    return -1;
  }
  const bool maze_cycle = parameter.value("maze_cycle", false);
  const bool lock_tcp_z = parameter.value("lock_tcp_z", maze_cycle);
  const double automatic_lift_m = parameter.value("automatic_lift_m", 0.02);
  const double lift_tolerance_m = parameter.value("lift_tolerance_m", 0.001);
  const double lift_velocity_tolerance_mps =
      parameter.value("lift_velocity_tolerance_mps", 0.005);
  const int lift_hold_cycles = parameter.value("lift_hold_cycles", 200);
  const double leader_initialization_speed_factor =
      parameter.value("leader_initialization_speed_factor", 0.05);
  const double tcp_z_stiffness =
      parameter.value("tcp_z_stiffness", 1000.0);
  const double tcp_z_damping = parameter.value(
      "tcp_z_damping", 2.0 * std::sqrt(tcp_z_stiffness));
  if (maze_cycle && (!lock_tcp_z || automatic_lift_m <= 0.0 ||
                     lift_tolerance_m <= 0.0 ||
                     lift_velocity_tolerance_mps <= 0.0 || lift_hold_cycles <= 0 ||
                     leader_initialization_speed_factor <= 0.0 ||
                     leader_initialization_speed_factor > 1.0 ||
                     tcp_z_stiffness <= 0.0 || tcp_z_damping < 0.0 ||
                     !std::isfinite(tcp_z_stiffness) ||
                     !std::isfinite(tcp_z_damping)))
  {
    std::cerr << "invalid maze lift or TCP z controller configuration"
              << std::endl;
    return -1;
  }

  g_recording_base_dir = base_dir;
  g_recording_trial_name = trial_name;
  g_next_episode_index = episode_idx;
  g_record_data_enabled = leadorfollow == "f" && record_data;
  g_record_camera_enabled = leadorfollow == "f" && record_camera;
  g_recording_camera_serials = camera_serials;
  g_recording_camera_enabled = camera_enabled;
  g_recording_enabled_camera_count = enabled_camera_count;
  g_automatic_lift_m = automatic_lift_m;
  double gain_tau_ld;
  double gain_dq_l;
  double gain_tau_f;

  if (tau_ext_feedback)
  {
    gain_tau_ld = 1;
    gain_dq_l = -1;
    gain_tau_f = 1;
  }
  else
  {
    gain_tau_ld = -1;
    gain_dq_l = 1;
    gain_tau_f = -1;
  }

  // position drift compensation
  double K_drift = parameter["k_pos_drift"];
  std::array<double, 7> position_error = {{0, 0, 0, 0, 0, 0, 0}};

  // passivity shortage augmentation
  double dissipation = 0.0;
  double shortage = 0.0;
  double eta = parameter["eta_passivity_shortage"];

  const double run_time = parameter["run_time"];
  // Set print rate for comparing commanded vs. measured torques.
  const double print_rate = parameter["print_rate"];

  // Initialize data fields for the print thread.
  struct
  {
    std::mutex mutex;
    bool has_data;
    double pandatime;
    std::array<double, 7> tau_d_last;
    std::array<double, 7> q_msr;
    std::array<double, 7> tau_ext;
    std::array<double, 7> q_remote_delta;
    std::array<double, 6> f_ext;
    std::array<double, 7> dq_local;
    std::array<double, 7> q_des;
    std::array<double, 7> dq_remote;
    std::array<double, 7> tau_c;
    std::array<double, 7> tau_remote;
    double E_F_in_delayed;
    double E_L_in_delayed;
    double E_L_in;
    double E_L_out;
    double E_F_in;
    double E_F_out;
    double alpha;
    double beta;

  } print_data{};
  std::atomic<bool> running{true};

  // recording data
  // v2 prepends host_steady_timestamp_ns to the legacy 29 columns. Camera
  // timestamp CSV files use the same clock for exact nearest-neighbour joins.
  std::array<double, 7> q_local_delta{0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> dq_local = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> q_remote_delta = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> dq_remote = {0, 0, 0, 0, 0, 0, 0};
  std::array<double, 7> tau_ext = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> tau_remote = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 6> f_local = {{0, 0, 0, 0, 0, 0}};
  std::array<double, 6> f_remote = {0, 0, 0, 0, 0, 0};
  double E_F_in_delayed = 0;
  double E_L_in_delayed = 0;
  double alpha = 0;
  double beta = 0;

  double pandatime = 0.0;
  double remotetime = 0.0;

  // udpwithremote thread;
  send_data Data2Send;
  recv_data Data2Recv;
  std::unique_ptr<franka::Gripper> gripper_ptr;

  Data2Send.stop_code = 0.0;
  Data2Recv.stop_code = 0.0;
  g_stop_requested.store(false);
  g_record_active.store(false);
  g_record_started.store(false);
  g_record_start_time_ns.store(0);
  g_episode_phase.store(kResetOrAlign);
  g_lift_target_z.store(std::numeric_limits<double>::quiet_NaN());

  std::thread t_send;
  std::thread t_recv;
  std::thread t_gripper;
  std::thread t_initialization_heartbeat;
  std::atomic<bool> initialization_heartbeat_running{false};
  int exit_code = 0;
  bool control_diagnostics_valid = false;
  std::array<double, 7> last_q_measured = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> last_q_desired = {{0, 0, 0, 0, 0, 0, 0}};
  std::array<double, 7> last_q_remote_delta = {{0, 0, 0, 0, 0, 0, 0}};
  t_send = std::thread(udpwithremote_send, std::ref(Data2Send), std::ref(running));
  t_recv = std::thread(udpwithremote_recv, std::ref(Data2Recv), std::ref(running));

  try
  {
    // Connect to robot.
    // XanMod PREEMPT_RT kernels may not expose /sys/kernel/realtime, which
    // causes libfranka to reject them even when FIFO real-time scheduling is
    // available. Real-time capability and user permissions must be verified
    // on the controller PC before using kIgnore.
    franka::Robot robot(robot_ip, franka::RealtimeConfig::kIgnore);
    try
    {
      setDefaultBehavior(robot);
    }
    catch (const franka::Exception &ex)
    {
      // print exception
      std::cout << ex.what() << std::endl;
      robot.automaticErrorRecovery();
      std::cout << "error recover." << std::endl;
      setDefaultBehavior(robot);
    }

    franka::RobotState initial_state = robot.readOnce();

    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    Eigen::Vector3d position_ini(initial_transform.translation());
    Eigen::Quaterniond orientation_ini(initial_transform.linear());

    std::cout << "q0=" << initial_state.q << std::endl;
    std::cout << "x0=" << position_ini << std::endl;

    {
      std::lock_guard<std::mutex> lock(Data2Send.mutex);
      Data2Send.q_local_absolute = initial_state.q;
      Data2Send.robot_state_valid = 1.0;
      Data2Send.episode_phase = static_cast<double>(kResetOrAlign);
      send_allowed = true;
    }
    cv_send.notify_one();

    // The follower may enter its torque callback while the leader is still
    // homing or moving. Keep sending the latest initialization state so its
    // communication watchdog sees a live peer throughout this sequence.
    if (leadorfollow == "l")
    {
      initialization_heartbeat_running.store(true);
      t_initialization_heartbeat = std::thread([&]()
      {
        while (running && initialization_heartbeat_running.load())
        {
          {
            std::lock_guard<std::mutex> lock(Data2Send.mutex);
            send_allowed = true;
          }
          cv_send.notify_one();
          std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
      });
    }

    bool follower_initially_grasped = false;
    std::cout << "[Gripper Init] Connecting to gripper at " << robot_ip << "..." << std::endl;
    gripper_ptr = std::make_unique<franka::Gripper>(robot_ip);
    std::cout << "[Gripper Init] Connected. Starting homing..." << std::endl;
    try
    {
      if (!gripper_ptr->homing())
      {
        throw std::runtime_error("homing returned false");
      }
    }
    catch (const std::exception &ex)
    {
      throw std::runtime_error(std::string("[Gripper Init] Homing failed: ") + ex.what());
    }
    std::cout << "[Gripper Init] Homing succeeded." << std::endl;
    if (leadorfollow == "l")
    {
      const franka::GripperState leader_gripper_state = gripper_ptr->readOnce();
      std::cout << "[Leader Gripper] Homed and left open at width "
                << leader_gripper_state.width << " m." << std::endl;
    }
    else
    {
      const franka::GripperState grasp_state = gripper_ptr->readOnce();
      follower_initially_grasped = false;
      std::cout << "[Follower Gripper] Homed. Current width "
                << grasp_state.width << " m; initial grasp skipped." << std::endl;
    }
    std::this_thread::sleep_for(std::chrono::seconds(3));

    if (leadorfollow == "l")
    {
      std::cout << "[Maze Init] Waiting for the follower's current joint pose."
                << std::endl;
      const auto initialization_deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(60);
      while (running && std::chrono::steady_clock::now() < initialization_deadline)
      {
        bool follower_state_ready = false;
        {
          std::lock_guard<std::mutex> lock(Data2Recv.mutex);
          follower_state_ready = Data2Recv.robot_state_valid > 0.5;
        }
        if (follower_state_ready)
        {
          break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }

      std::array<double, 7> follower_q;
      {
        std::lock_guard<std::mutex> lock(Data2Recv.mutex);
        if (Data2Recv.robot_state_valid <= 0.5)
        {
          throw std::runtime_error(
              "timed out waiting for the follower joint pose");
        }
        follower_q = Data2Recv.q_remote_absolute;
      }
      for (double joint : follower_q)
      {
        if (!std::isfinite(joint))
        {
          throw std::runtime_error("received a non-finite follower joint pose");
        }
      }

      std::cout << "[Maze Init] Moving leader to follower q=" << follower_q
                << std::endl;
      MotionGenerator initialization_motion(
          leader_initialization_speed_factor, follower_q);
      robot.control([&](const franka::RobotState &state,
                        franka::Duration period) -> franka::JointPositions
      {
        {
          std::lock_guard<std::mutex> lock(Data2Send.mutex);
          Data2Send.q_local_absolute = state.q;
          Data2Send.robot_state_valid = 1.0;
        }
        return initialization_motion(state, period);
      });
      initial_state = robot.readOnce();
      std::cout << "[Maze Init] Leader reached the follower pose. Close the "
                   "leader gripper to start the first lift."
                << std::endl;
    }
    else
    {
      std::cout << "[Maze Init] Holding the current follower pose while the "
                   "leader initializes."
                << std::endl;
    }

    t_gripper = std::thread(gripperControl, std::ref(Data2Send), std::ref(Data2Recv),
                            std::ref(running), std::ref(*gripper_ptr), leadorfollow,
                            follower_initially_grasped, gripper_grasp_force);

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();

    initial_state = robot.readOnce();
    // Bias torque sensor
    std::cout << "q=" << initial_state.q << std::endl;
    std::cout << std::endl;
    // std::cin.ignore();

    // Set gains for the joint impedance control.
    // Stiffness
    // const std::array<double, 7> k_gains = {{600.0, 600.0, 600.0, 600.0, 250.0, 150.0, 100.0}};
    // const std::array<double, 7> d_gains = {{2 * sqrt(k_gains[0]), 2 * sqrt(k_gains[1]), 2 * sqrt(k_gains[2]),
    //                                        2 * sqrt(k_gains[3]), 2 * sqrt(k_gains[4]), 2 * sqrt(k_gains[5]),
    //                                        2 * sqrt(k_gains[6])}};

    // Dynamics of the robot, identified using Mario Tröbinger's method
    Eigen::VectorXd Xb(66);
    for (int index = 0; index < parameter["Xb"].size(); ++index)
    {
      Xb[index] = parameter["Xb"][index];
    }
    Dynamics dyn(M_PI, 0.0, Xb); // Robot dynamics
    const Eigen::Matrix<double, 7, 7> friction = dyn.get_Fv();

    //////////////////////////////////////////////////////////////////////////////////////////
    ///////////         TDPA stuff begin         ///////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

    // Leader Side
    TDPA_Leader leaderPC;
    leaderPC.init();

    double dq_local_double[7], tau_remote_double[7], tau_local_double[7];
    double E_F_in_delayed;
    double E_L_in, E_L_out, E_L_diss;

    // Follower Side
    TDPA_Follower followerPC;
    followerPC.init();

    // MassSpringFilter PassiveLPFilter(0.001, 1000, 0.001, initial_state.q.data());

    double dq_remote_double[7], tau_c_old_double[7], dq_des_double[7];
    double E_L_in_delayed;
    double E_F_in, E_F_out, E_F_diss;
    std::array<double, 7> tau_c = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_c_no_mod_dq = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> tau_diff = {{0, 0, 0, 0, 0, 0, 0}};

    // Tracking controller parameters
    std::array<double, 7> k_gains = {{0, 0, 0, 0, 0, 0, 0}};
    std::array<double, 7> d_gains = {{0, 0, 0, 0, 0, 0, 0}};
    Eigen::Matrix<double, 7, 7> damping = Eigen::Matrix<double, 7, 7>::Zero();

    for (int index = 0; index < parameter["k_gains"].size(); ++index)
    {
      k_gains[index] = parameter["k_gains"][index];
      d_gains[index] = 2 * sqrt(k_gains[index]);
      damping(index, index) = d_gains[index];
    }

    std::cout << "TDPA initialize done" << std::endl;

    //////////////////////////////////////////////////////////////////////////////////////////
    ///////////         TDPA stuff end         ///////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////

    std::cout << "[Recording] Waiting for leader gripper close." << std::endl;

    ////////////////////// Define callback for the joint torque control
    /// loop.//////////////////////////////////////////////////////////////////////

    int leader_stop_ack_count = 0;
    std::array<double, 7> teleop_joint_baseline = initial_state.q;
    bool teleop_enabled = false;
    int lift_settle_cycles = 0;
    int remote_episode_phase = kResetOrAlign;
    std::function<franka::Torques(const franka::RobotState &, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState &state,
                                         franka::Duration period /*period*/) -> franka::Torques
    {
      // Read current coriolis terms from model.
      // std::cerr << "1111" << std::endl;
      pandatime += period.toSec();

      if (g_signal_stop_requested)
      {
        g_record_active.store(false);
        throw std::runtime_error(
            "termination signal received; flushing committed recording data");
      }

      if (leadorfollow == "f" && g_camera_capture_failed.load())
      {
        g_record_active.store(false);
        throw std::runtime_error(
            "RGB-D capture failed; stopping the episode with committed data preserved");
      }

      // Once UDP communication has started, stop rather than reusing stale
      // motion commands. Starting the follower before the leader is allowed:
      // the watchdog is armed only after the first packet arrives.
      if (Data2Recv.has_received.load(std::memory_order_acquire) &&
          !g_stop_requested.load(std::memory_order_acquire))
      {
        constexpr int64_t udp_watchdog_timeout_ns = 100'000'000; // 100 ms
        const int64_t packet_age_ns =
            steady_time_ns() - Data2Recv.last_receive_time_ns.load(std::memory_order_acquire);
        if (packet_age_ns > udp_watchdog_timeout_ns)
        {
          throw std::runtime_error("UDP watchdog timeout: no packet received for more than 100 ms");
        }
      }

      static double teleop_active_recv = 0.0;
      // static int leader_stop_ack_count = 0;

      std::array<double, 7> coriolis_array = model.coriolis(state);
      std::array<double, 7> gravity_array = model.gravity(state);
      std::array<double, 42> jacobian_array =
          model.zeroJacobian(franka::Frame::kEndEffector, state);
      std::array<double, 49> mass_array = model.mass(state);
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 7>> mass(mass_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> gravity(gravity_array.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> tau_J_d_eig(state.tau_J_d.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq_eig(state.dq.data());
      const Eigen::Matrix<double, 6, 7> tcp_jacobian =
          Kinematics::ComputeJacobian(state.q.data(), kPandaHandTcpOffsetM);
      const Eigen::Matrix<double, 1, 7> z_jacobian = tcp_jacobian.row(2);
      const Eigen::Matrix4d tcp_pose =
          Kinematics::ForwardKinematics(state.q.data(), kPandaHandTcpOffsetM);
      // external wrench acting on stiffness frame, expressed relative to the stiffness frame.
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> K_F_ext_hat_K(state.K_F_ext_hat_K.data());

      // ************************* POCO *************************
      if (vis)
      {
        m_json.at("F_ext") = {};
        for (auto &i : state.K_F_ext_hat_K)
        {
          m_json["F_ext"].push_back(i);
        }
        m_json.at("Q") = {};
        for (auto &i : state.q)
        {
          m_json["Q"].push_back(i);
        }
        m_msg = m_json.dump();
        try
        {
          m_dgs.sendBytes(m_msg.data(), m_msg.size());
        }
        catch (std::exception &e)
        {
          std::cout << e.what() << std::endl;
          std::cout << "The corresponding UDP receiver with localhost:6666 is not activated; Otherwise, the publisher won't start!" << std::endl;
          vis = false;
        }
      }

      // ************************* POCO *************************

      for (size_t i = 0; i < 7; i++)
      {
        tau_ext[i] = -state.tau_ext_hat_filtered[i];
      }
      f_local = state.O_F_ext_hat_K;
      dq_local = state.dq;

      // ************************ teleop_active *****************
      static double teleop_active = 0.0;
      static int on_cnt = 0;
      static int off_cnt = 0;

      const double dq_on = 0.05;
      const double dq_off = 0.03;
      const int N_on = 30;
      const int N_off = 50;

      double dq_norm = 0.0;
      for (int i = 0; i < 7; i++)
      {
        dq_norm += dq_local[i] * dq_local[i];
      }
      dq_norm = std::sqrt(dq_norm);

      if (dq_norm > dq_on)
      {
        on_cnt++;
        off_cnt = 0;
        if (on_cnt >= N_on)
          teleop_active = 1.0; // move continuously for 30ms
      }
      else if (dq_norm < dq_off)
      {
        off_cnt++;
        on_cnt = 0;
        if (off_cnt >= N_off)
          teleop_active = 0.0;
      }
      else
      {
        on_cnt = 0;
        off_cnt = 0;
      }

      //************************* teleop_active *****************

      if (leadorfollow == "l")
      {
        if (Data2Recv.mutex.try_lock())
        {
          remotetime = Data2Recv.remotetime;
          q_remote_delta = Data2Recv.q_remote_delta;
          dq_remote = Data2Recv.dq_remote;
          tau_remote = Data2Recv.tau_remote;
          f_remote = Data2Recv.f_remote;
          E_F_in_delayed = Data2Recv.energy;
          remote_episode_phase = static_cast<int>(Data2Recv.episode_phase);

          Data2Recv.mutex.unlock();
        }
      }
      else
      {
        if (Data2Recv.mutex.try_lock())
        {
          remotetime = Data2Recv.remotetime;
          q_remote_delta = Data2Recv.q_remote_delta;
          dq_remote = Data2Recv.dq_remote;
          tau_remote = Data2Recv.tau_remote;
          f_remote = Data2Recv.f_remote;
          E_L_in_delayed = Data2Recv.energy;
          teleop_active_recv = Data2Recv.teleop_active;
          remote_episode_phase = static_cast<int>(Data2Recv.episode_phase);

          Data2Recv.mutex.unlock();
        }
      }

      if (maze_cycle && g_episode_phase.load() == kCloseDetected &&
          remote_episode_phase >= kCloseDetected &&
          remote_episode_phase <= kLifting)
      {
        teleop_joint_baseline = state.q;
        teleop_enabled = true;
        lift_settle_cycles = 0;
        g_lift_target_z.store(tcp_pose(2, 3) + automatic_lift_m);
        g_episode_phase.store(kLifting);
      }

      std::array<double, 7> q_des;
      for (size_t i = 0; i < 7; i++)
      {
        q_local_delta[i] = teleop_enabled
                               ? state.q[i] - teleop_joint_baseline[i]
                               : 0.0;
      }

      //////////////////////////////////////////////////////////////////////////////////////////
      ///////////         TDPA stuff begin         ///////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////

      if (leadorfollow == "l")
      {
        memcpy(dq_local_double, state.dq.data(), 7 * sizeof(double));
        for (size_t i = 0; i < 7; i++)
        {
          dq_local_double[i] = gain_dq_l * dq_local_double[i];
        }
        memcpy(tau_remote_double, tau_remote.data(), 7 * sizeof(double));

        if (period.toSec() == 0)
          period = franka::Duration(1);

        leaderPC.energyObserver(dq_local_double, tau_remote_double, period.toSec());
        memcpy(tau_local_double, tau_remote_double, 7 * sizeof(double));

        // shortage computation as integral of robot energy dissipation, on leader side only the robot friction is used
        dissipation = dq_eig.transpose() * friction * dq_eig;
        shortage += period.toSec() * eta * dissipation;

        if (TDPA_active)
        {

          leaderPC.energyController(tau_local_double, dq_local_double, E_F_in_delayed + shortage, period.toSec());
        }

        for (int i = 0; i < 7; i++)
        {
          tau_diff[i] = tau_remote_double[i] - tau_local_double[i];
        }

        alpha = leaderPC.getAlpha();
        E_L_in = leaderPC.getInputEnergyFlow();
        E_L_out = leaderPC.getOutputEnergyFlow();
        E_L_diss = leaderPC.getDissipatedEnergyFlow();
      }
      else
      {
        for (size_t i = 0; i < 7; i++)
        {
          dq_remote[i] = dq_remote[i] - K_drift * position_error[i];
        }

        memcpy(dq_remote_double, dq_remote.data(), 7 * sizeof(double));
        if (tau_ext_feedback)
        {
          memcpy(tau_c_old_double, tau_ext.data(), 7 * sizeof(double));
        }
        else
        {
          memcpy(tau_c_old_double, tau_c.data(), 7 * sizeof(double));
        }
        for (size_t i = 0; i < 7; i++)
        {
          tau_c_old_double[i] = gain_tau_f * tau_c_old_double[i];
        }

        if (period.toSec() == 0)
          period = franka::Duration(1);

        followerPC.energyObserver(dq_remote_double, tau_c_old_double, period.toSec());
        memcpy(dq_des_double, dq_remote_double, 7 * sizeof(double));

        // shortage computation as integral of robot energy dissipation
        dissipation = dq_eig.transpose() * (damping + friction) * dq_eig;
        shortage += period.toSec() * eta * dissipation;

        if (TDPA_active)
        {
          followerPC.energyController(dq_des_double, tau_c_old_double, E_L_in_delayed + shortage, period.toSec());
        }

        for (size_t i = 0; i < 7; i++)
        {
          // Re-zero both robots at every close event. Before the first close,
          // hold the follower still while the leader initializes to its pose.
          // The peer publishes its phase and re-zeroed joint delta in one UDP
          // packet. Hold this robot at the freshly captured baseline until
          // that packet arrives, so a previous episode's delta cannot be
          // applied during the close handshake.
          const bool remote_baseline_ready =
              g_episode_phase.load() != kLifting ||
              remote_episode_phase >= kLifting;
          q_des[i] = teleop_enabled && remote_baseline_ready
                         ? teleop_joint_baseline[i] + q_remote_delta[i]
                         : teleop_joint_baseline[i];
          if (!teleop_enabled || !remote_baseline_ready)
          {
            dq_des_double[i] = 0.0;
          }
          position_error[i] = q_local_delta[i] - q_remote_delta[i];
        }

        last_q_measured = state.q;
        last_q_desired = q_des;
        last_q_remote_delta = q_remote_delta;
        control_diagnostics_valid = true;

        for (size_t i = 0; i < 7; i++)
        {
          tau_c[i] = k_gains[i] * (q_des[i] - state.q[i]) +
                     0.5 * d_gains[i] * (dq_des_double[i] - dq_local[i]) + 0 * dq_local[i] +
                     0 * coriolis[i];
          tau_c_no_mod_dq[i] = k_gains[i] * (q_des[i] - state.q[i]) +
                               0.5 * d_gains[i] * (dq_remote_double[i] - dq_local[i]) + 0 * dq_local[i] +
                               0 * coriolis[i];
          tau_diff[i] = tau_c_no_mod_dq[i] - tau_c[i];
        }

        beta = followerPC.getBeta();
        E_F_in = followerPC.getInputEnergyFlow();
        E_F_out = followerPC.getOutputEnergyFlow();
        E_F_diss = followerPC.getDissipatedEnergyFlow();
      }

      //////////////////////////////////////////////////////////////////////////////////////////
      ///////////         TDPA stuff end         ///////////////////////////
      //////////////////////////////////////////////////////////////////////////////////////////

      std::array<double, 7> tau_d_calculated;

      if (leadorfollow == "l")
      {
        for (size_t i = 0; i < 7; i++)
        {
          // if (!TDPA_active)
          // tau_d_calculated[i] = 0.0 * coriolis[i] - 1.0 * tau_remote[i] - 2 * dq_local[i];
          // else
          tau_d_calculated[i] = 0.0 * coriolis[i] + gain_tau_ld * tau_local_double[i] - 0 * dq_local[i];
        }
      }
      else if (leadorfollow == "f")
      {
        tau_d_calculated = tau_c;
      }

      int local_episode_phase = g_episode_phase.load();
      if (leadorfollow == "l" && local_episode_phase == kLiftReady &&
          remote_episode_phase == kRecording)
      {
        g_episode_phase.store(kRecording);
        local_episode_phase = kRecording;
      }
      const bool hold_lift_height = maze_cycle && lock_tcp_z &&
                                    local_episode_phase >= kLifting;
      if (hold_lift_height)
      {
        const double target_z = g_lift_target_z.load();
        const double z_velocity = (z_jacobian * dq_eig)(0);
        const double z_error = target_z - tcp_pose(2, 3);
        const double denominator = z_jacobian.squaredNorm() + 1e-9;
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> nominal_torque(
            tau_d_calculated.data());
        Eigen::Matrix<double, 7, 1> constrained_torque = nominal_torque;
        constrained_torque -= z_jacobian.transpose() *
                              ((z_jacobian * nominal_torque)(0) / denominator);
        const double z_force = tcp_z_stiffness * z_error -
                               tcp_z_damping * z_velocity;
        constrained_torque += z_jacobian.transpose() * z_force;
        Eigen::VectorXd::Map(tau_d_calculated.data(), 7) = constrained_torque;

        if (local_episode_phase == kLifting)
        {
          if (std::abs(z_error) <= lift_tolerance_m &&
              std::abs(z_velocity) <= lift_velocity_tolerance_mps)
          {
            ++lift_settle_cycles;
            if (lift_settle_cycles >= lift_hold_cycles)
            {
              g_episode_phase.store(kLiftReady);
            }
          }
          else
          {
            lift_settle_cycles = 0;
          }
        }
      }

      std::array<double, 7> tau_d_rate_limited =
          franka::limitRate(franka::kMaxTorqueRate, tau_d_calculated, state.tau_J_d);

      bool send_data_updated = false;
      {
        // Never wait for the non-real-time UDP sender from inside the 1 kHz
        // robot callback. If the sender is copying a packet, skip this cycle
        // and publish the newest state on the next available cycle.
        std::unique_lock<std::mutex> lck_send(Data2Send.mutex, std::try_to_lock);
        if (lck_send.owns_lock())
        {

          if (leadorfollow == "l")
          {
            Data2Send.pandatime = pandatime;
            Data2Send.q_local_delta = q_local_delta;
            Data2Send.q_local_absolute = state.q;
            Data2Send.robot_state_valid = 1.0;
            Data2Send.dq_local = dq_local;
            Data2Send.teleop_active = teleop_active;
            if (tau_ext_feedback)
              Data2Send.tau_local = tau_ext;
            else
              Data2Send.tau_local = tau_c;
            Data2Send.f_local = f_local;
            Data2Send.energy = E_L_in;

            if (Data2Recv.stop_code == 1.0)
            {
              Data2Send.stop_code = 2.0;
              leader_stop_ack_count++;
            }
            else if (leader_stop_ack_count == 0)
            {
              Data2Send.stop_code = 0.0;
            }
          }
          else
          {
            Data2Send.pandatime = pandatime;
            Data2Send.q_local_delta = q_local_delta;
            Data2Send.q_local_absolute = state.q;
            Data2Send.robot_state_valid = 1.0;
            Data2Send.dq_local = dq_local;
            if (tau_ext_feedback)
              Data2Send.tau_local = tau_ext;
            else
            {
              Data2Send.tau_local[0] = tau_c[0];
              Data2Send.tau_local[1] = tau_c[1];
              Data2Send.tau_local[2] = tau_c[2];
              Data2Send.tau_local[3] = tau_c[3];
              Data2Send.tau_local[4] = tau_c[4];
              Data2Send.tau_local[5] = tau_c[5];
              Data2Send.tau_local[6] = tau_c[6];
            }
            Data2Send.f_local = f_local;
            Data2Send.energy = E_F_in;
          }

          Data2Send.episode_phase =
              static_cast<double>(g_episode_phase.load());

          send_allowed = true;
          send_data_updated = true;
        }
      }
      if (send_data_updated)
      {
        cv_send.notify_one();
      }

      // Logging Data
      if (leadorfollow == "f" && g_record_data_enabled &&
          g_record_active.load())
      {
        if(g_record_mutex.try_lock())
        {
          double follower_width = g_follower_gripper_width.load();

          std::array<double, 7> leader_q;
          for (int i = 0; i < 7; ++i){
            leader_q[i] = teleop_joint_baseline[i] + q_remote_delta[i];
          }
          // Eigen::VectorXd row(15);
          // int c = 0;

          // row(c++) = teleop_active_recv;

          // for (int i = 0; i < 7; ++i) row(c++) = state.q[i];

          // row(c++) = follower_width;

          // for (int i = 0; i < 6; ++i) row(c++) = state.K_F_ext_hat_K[i];

          // rec->addRow(row);

          // rec->addToRec(index);
          // rec->addToRec(pandatime);
          // rec->addToRec(remotetime);
          g_episode_recorder->addToRec(static_cast<double>(steady_time_ns()));
          g_episode_recorder->addToRec(teleop_active_recv);
          g_episode_recorder->addToRec(state.q);
          g_episode_recorder->addToRec(follower_width);
          g_episode_recorder->addToRec(state.K_F_ext_hat_K);
          // rec->addToRec(q_local_delta); // q_local_delta
          // rec->addToRec(q_remote_delta);
          // rec->addToRec(dq_local);
          // rec->addToRec(dq_remote);
          g_episode_recorder->addToRec(leader_q); // leader joint trajectory
          g_episode_recorder->addToRec(tau_ext);
          // rec->addToRec(tau_c.data(), 7);
          // rec->addToRec(tau_remote);
          // rec->addToRec(f_local);
          // rec->addToRec(f_remote);
          // rec->addToRec(E_L_in);
          // rec->addToRec(E_L_out);
          // rec->addToRec(E_F_in_delayed);
          // rec->addToRec(E_L_diss);
          // rec->addToRec(E_F_in);
          // rec->addToRec(E_F_out);
          // rec->addToRec(E_L_in_delayed);
          // rec->addToRec(E_F_diss);
          // rec->addToRec(alpha);
          // rec->addToRec(beta);
          // rec->addToRec(position_error);
          // rec->addToRec(shortage);
          // rec->addToRec(tau_diff);
          // rec->addToRec(position);

        
          g_episode_recorder->next();

          g_record_mutex.unlock();
        }
      }

      // ***********leader recieve the stop request, exit*************
      franka::Torques zerotorque = tau_d_rate_limited;

      // Give the UDP sender enough control cycles to transmit stop_code 2
      // before the leader stops and tears down its communication threads.
      if (leadorfollow == "l" && g_stop_requested.load() && leader_stop_ack_count >= 200)
      {
        running = false;
        send_allowed = true;
        cv_send.notify_all();
        return franka::MotionFinished(zerotorque);
      }

      if (leadorfollow == "f" && Data2Recv.stop_code == 2.0)
      {
        g_record_active.store(false);
        running = false;
        send_allowed = true;
        cv_send.notify_all();
        return franka::MotionFinished(zerotorque);
      }

      return tau_d_rate_limited;
    };

    initialization_heartbeat_running.store(false);
    if (t_initialization_heartbeat.joinable())
    {
      t_initialization_heartbeat.join();
    }
    robot.control(impedance_control_callback);
  }
  catch (const std::exception &ex)
  {

    std::cerr << "[TelePanda] control stopped: " << ex.what() << std::endl;
    if (control_diagnostics_valid)
    {
      std::cerr << "[TelePanda] last q measured      = " << last_q_measured << std::endl;
      std::cerr << "[TelePanda] last q desired       = " << last_q_desired << std::endl;
      std::cerr << "[TelePanda] last leader q delta  = " << last_q_remote_delta << std::endl;
    }
    exit_code = 1;
    g_record_active.store(false);

    if (leadorfollow == "f")
    {
      uint16_t PORTSend = 5001;

      int sockfd;
      struct sockaddr_in servaddr;
      memset(&servaddr, 0, sizeof(servaddr));

      if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
      {
        perror("socket creation failed");
      }
      else
      {
        servaddr.sin_family = AF_INET;
        servaddr.sin_port = htons(PORTSend);
        servaddr.sin_addr.s_addr = inet_addr(IP_remote);

        // Keep emergency stop packets compatible with the normal 41-double
        // UDP protocol. A legacy packet is rejected by the peer and
        // would make the peer report a watchdog timeout instead of stopping.
        const int NoDatatosend = 41;
        double msg2send[NoDatatosend] = {0};

        msg2send[29] = 1.0;
        msg2send[30] = g_follower_gripper_width.load();
        msg2send[31] = 0.0;
        msg2send[32] = static_cast<double>(kResetOrAlign);

        for (int k = 0; k < 20; k++)
        {
          sendto(sockfd, msg2send, sizeof(msg2send), MSG_CONFIRM, (const struct sockaddr *)&servaddr,
                 sizeof(servaddr));
        }

        close(sockfd);
      }
    }

    running = false;
    send_allowed = true;
    cv_send.notify_all();
  }

  if (t_gripper.joinable())
  {
    t_gripper.join();
  }

  initialization_heartbeat_running.store(false);
  if (t_initialization_heartbeat.joinable())
  {
    t_initialization_heartbeat.join();
  }

  t_send.detach();
  t_recv.detach();

  if (leadorfollow == "f")
  {
    g_camera_episode_running.store(false);
    for (auto &camera_thread : g_camera_threads)
    {
      if (camera_thread.joinable())
      {
        camera_thread.join();
      }
    }
    std::lock_guard<std::mutex> lock(g_record_mutex);
    g_episode_recorder.reset();
  }
  return exit_code;
}

void gripperControl(send_data &Data2Send, recv_data &Data2Recv, std::atomic<bool> &running, franka::Gripper &gripper,
                    const std::string &leadorfollow, bool initially_grasped,
                    double grasp_force)
{
  (void)Data2Send;
  (void)initially_grasped;
  constexpr double close_threshold = 0.005;
  constexpr double open_threshold = 0.02;
  constexpr int required_state_samples = 5;
  bool close_armed = false;
  bool cycle_in_progress = false;
  bool episode_open = false;
  int open_samples = 0;
  int close_samples = 0;
  int leader_width_print_count = 0;

  auto join_episode_cameras = [&]()
  {
    g_camera_episode_running.store(false);
    for (auto &camera_thread : g_camera_threads)
    {
      if (camera_thread.joinable())
      {
        camera_thread.join();
      }
    }
  };

  auto finish_episode = [&](bool requested_complete)
  {
    if (!episode_open)
    {
      return;
    }
    g_record_active.store(false);
    join_episode_cameras();
    {
      std::lock_guard<std::mutex> lock(g_record_mutex);
      g_episode_recorder.reset();
    }

    const bool robot_data_complete =
        !g_record_data_enabled ||
        (fs::is_regular_file(g_current_episode_dir / "DATA_follower.m") &&
         fs::file_size(g_current_episode_dir / "DATA_follower.m") > 0);
    bool camera_data_complete = !g_record_camera_enabled ||
                                !g_camera_capture_failed.load();
    json recorded_cameras = json::array();
    json recorded_camera_serials = json::object();
    json committed_camera_frames = json::object();
    for (size_t camera_index = 0;
         camera_index < g_recording_camera_enabled.size(); ++camera_index)
    {
      if (!g_record_camera_enabled ||
          !g_recording_camera_enabled[camera_index])
      {
        continue;
      }
      const std::string camera_name = "cam" + std::to_string(camera_index + 1);
      const uint64_t frames = g_camera_committed_frames[camera_index].load();
      recorded_cameras.push_back(camera_name);
      recorded_camera_serials[camera_name] =
          g_recording_camera_serials[camera_index];
      committed_camera_frames[camera_name] = frames;
      camera_data_complete = camera_data_complete && frames > 0;
    }
    json manifest = {
        {"format", "maze-rgbd-recording-v3"},
        {"complete", requested_complete && g_record_started.load() &&
                         robot_data_complete && camera_data_complete},
        {"role", "follower"},
        {"record_data", g_record_data_enabled},
        {"record_camera", g_record_camera_enabled},
        {"recording_start", "after_both_robots_lifted"},
        {"recording_started", g_record_started.load()},
        {"recording_start_host_steady_timestamp_ns",
         g_record_start_time_ns.load()},
        {"automatic_lift_m", g_automatic_lift_m},
        {"fixed_tcp_z_m", g_lift_target_z.load()},
        {"fixed_tcp_frame", "panda_hand_tcp"},
        {"robot_columns", kRobotRecordColumns},
        {"robot_timestamp_column", "host_steady_timestamp_ns"},
        {"camera_timestamp_clock", "host_steady_timestamp_ns"},
        {"depth_encoding", "uint16_z16_aligned_to_color"},
        {"recorded_cameras", recorded_cameras},
        {"camera_serials", recorded_camera_serials},
        {"committed_camera_frames", committed_camera_frames}};
    const fs::path temporary = g_current_episode_dir / "recording_manifest.json.tmp";
    const fs::path final_path = g_current_episode_dir / "recording_manifest.json";
    std::ofstream stream(temporary);
    stream << manifest.dump(2) << std::endl;
    stream.close();
    fs::rename(temporary, final_path);
    std::cout << "[Recording] Episode " << g_next_episode_index
              << " saved at " << g_current_episode_dir << std::endl;
    ++g_next_episode_index;
    episode_open = false;
    g_current_episode_dir.clear();
    g_record_started.store(false);
    g_record_start_time_ns.store(0);
  };

  auto prepare_episode = [&]()
  {
    g_current_episode_dir = g_recording_base_dir / g_recording_trial_name /
                            format_episode_dir(g_next_episode_index);
    if (fs::exists(g_current_episode_dir))
    {
      throw std::runtime_error("refusing to overwrite existing episode " +
                               g_current_episode_dir.string());
    }
    fs::create_directories(g_current_episode_dir);
    episode_open = true;
    g_record_active.store(false);
    g_record_started.store(false);
    g_record_start_time_ns.store(0);
    g_camera_capture_failed.store(false);
    g_cameras_ready.store(0);
    for (auto &count : g_camera_committed_frames)
    {
      count.store(0);
    }
    if (g_record_data_enabled)
    {
      std::lock_guard<std::mutex> lock(g_record_mutex);
      g_episode_recorder = std::make_unique<Recorder>(
          kInitialRecordSeconds, kRobotRecordSampleSeconds,
          kRobotRecordColumns,
          (g_current_episode_dir / "DATA_follower").string());
    }
    if (g_record_camera_enabled)
    {
      g_camera_episode_running.store(true);
      for (int camera_index = 0; camera_index < 3; ++camera_index)
      {
        if (!g_recording_camera_enabled[camera_index])
        {
          continue;
        }
        g_camera_threads[camera_index] = std::thread(
            rgbd_camera_thread_func, camera_index + 1, std::ref(running),
            g_current_episode_dir.string(),
            g_recording_camera_serials[camera_index],
            std::ref(g_cameras_ready), std::ref(g_camera_capture_failed),
            std::ref(g_camera_committed_frames[camera_index]));
      }
      const auto deadline =
          std::chrono::steady_clock::now() + std::chrono::seconds(15);
      while (running &&
             g_cameras_ready.load() < g_recording_enabled_camera_count &&
             !g_camera_capture_failed.load() &&
             std::chrono::steady_clock::now() < deadline)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
      }
      if (g_camera_capture_failed.load() ||
          g_cameras_ready.load() != g_recording_enabled_camera_count)
      {
        throw std::runtime_error("failed to initialize all enabled RGB-D cameras");
      }
    }
  };

  try
  {
    while (running)
    {
      double target_width = 0.08;
      int remote_phase = kResetOrAlign;
      if (leadorfollow == "l")
      {
        target_width = gripper.readOnce().width;
        if (leader_width_print_count++ % 100 == 0)
        {
          std::cout << "Gripper width is:" << target_width << std::endl;
        }
        Data2Send.gripper_width.store(target_width, std::memory_order_release);
      }
      else
      {
        {
          std::lock_guard<std::mutex> lock(Data2Recv.mutex);
          target_width = Data2Recv.gripper_width;
          remote_phase = static_cast<int>(Data2Recv.episode_phase);
        }
        g_follower_gripper_width.store(gripper.readOnce().width);
      }

      if (!cycle_in_progress && target_width >= open_threshold)
      {
        if (++open_samples >= required_state_samples)
        {
          close_armed = true;
          open_samples = 0;
        }
      }
      else
      {
        open_samples = 0;
      }

      if (close_armed && !cycle_in_progress && target_width < close_threshold)
      {
        if (++close_samples >= required_state_samples)
        {
          close_samples = 0;
          close_armed = false;
          cycle_in_progress = true;
          bool local_close_ready = true;
          if (leadorfollow == "f")
          {
            // First close to locate the object's actual width. For objects
            // wider than 5 mm this probe is expected to return false, but its
            // final state gives us the contact width for a force-holding grasp.
            bool grasp_confirmed =
                gripper.grasp(0.0, 0.05, grasp_force, 0.005, 0.005);
            franka::GripperState grasp_state = gripper.readOnce();
            g_follower_gripper_width.store(grasp_state.width);
            if (!grasp_confirmed || !grasp_state.is_grasped)
            {
              constexpr double kMinimumObjectWidthM = 0.002;
              constexpr double kGripPreloadM = 0.002;
              if (grasp_state.width > kMinimumObjectWidthM)
              {
                const double holding_target_width =
                    std::max(0.0, grasp_state.width - kGripPreloadM);
                std::cout << "[Follower Gripper] Contact detected at width="
                          << grasp_state.width << " m; applying " << grasp_force
                          << " N holding grasp with target="
                          << holding_target_width << " m." << std::endl;
                grasp_confirmed = gripper.grasp(
                    holding_target_width, 0.02, grasp_force, 0.005, 0.005);
                grasp_state = gripper.readOnce();
                g_follower_gripper_width.store(grasp_state.width);
              }
            }

            if (!grasp_confirmed || !grasp_state.is_grasped)
            {
              local_close_ready = false;
              std::cerr << "[Follower Gripper] Object is not securely grasped; "
                           "lift cancelled. width="
                        << grasp_state.width
                        << " m, is_grasped=" << grasp_state.is_grasped
                        << ". Open and close the leader gripper to retry."
                        << std::endl;
            }
            else
            {
              std::cout << "[Follower Gripper] Object secured at width="
                        << grasp_state.width << " m with force=" << grasp_force
                        << " N." << std::endl;
              prepare_episode();
            }
          }
          if (local_close_ready)
          {
            g_episode_phase.store(kCloseDetected);
            std::cout << "[Maze] Close confirmed; waiting for both robots, then lifting "
                      << g_automatic_lift_m << " m."
                      << std::endl;
          }
        }
      }
      else if (target_width >= close_threshold)
      {
        close_samples = 0;
      }

      if (leadorfollow == "f" && cycle_in_progress &&
          g_episode_phase.load() == kLiftReady &&
          remote_phase >= kLiftReady && !g_record_active.load())
      {
        g_record_start_time_ns.store(steady_time_ns());
        g_record_started.store(true);
        g_record_active.store(true);
        g_episode_phase.store(kRecording);
        std::cout << "[Recording] Started episode " << g_next_episode_index
                  << " after both robots reached z=" << g_lift_target_z.load()
                  << " m." << std::endl;
      }

      const bool release_requested =
          target_width >= open_threshold ||
          (leadorfollow == "f" && remote_phase == kResetOrAlign);
      if (cycle_in_progress && release_requested)
      {
        if (++open_samples >= required_state_samples)
        {
          open_samples = 0;
          // End the episode and release the z constraint immediately. The
          // camera/recorder flush can take much longer than opening the hand,
          // so it must not delay the follower's physical release.
          g_record_active.store(false);
          g_episode_phase.store(kResetOrAlign);
          if (leadorfollow == "f")
          {
            gripper.stop();
            const double max_width = gripper.readOnce().max_width;
            if (!gripper.move(max_width, 0.1))
            {
              throw std::runtime_error("follower gripper failed to open after episode");
            }
            g_follower_gripper_width.store(gripper.readOnce().width);
            std::cout << "[Follower Gripper] Released on leader open command."
                      << std::endl;
            finish_episode(true);
          }
          cycle_in_progress = false;
          close_armed = true;
          std::cout << "[Maze] Episode ended. Manually reset with leader; close again for next episode."
                    << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(10));
    }
  }
  catch (const std::exception &e)
  {
    std::cerr << "Gripper control exception:" << e.what() << std::endl;
    running = false;
  }
  if (leadorfollow == "f" && episode_open)
  {
    finish_episode(false);
  }
}

bool movetoGrasp(franka::Gripper &gripper, double target_width,
                 bool &grasp_flag, bool &ever_grasped, double grasp_force)
{

  const double grasp_threshold = 0.005;
  // double hold_threshold = 0.02;
  const double release_threshold = 0.02;
  const double exit_threshold = 0.07;

  // double current_width = gripper.readOnce().width;
  // double target_width = gripper_width.load();

  // grasp

  if (!ever_grasped && !grasp_flag && target_width < grasp_threshold)
  {
    try
    {
      const double max_width = gripper.readOnce().max_width;
      if (gripper.grasp(0.0, 0.05, grasp_force, 0.0, max_width))
      {
        grasp_flag = true;
        ever_grasped = true;
        const double grasped_width = gripper.readOnce().width;
        std::cout << "[Follower Gripper] Grasp succeeded at width "
                  << grasped_width << " m with force " << grasp_force << " N."
                  << std::endl;
      }
      else
      {
        const franka::GripperState failed_grasp_state = gripper.readOnce();
        std::cerr << "[Follower Gripper] Failed to grasp an object with force "
                  << grasp_force << " N; width=" << failed_grasp_state.width
                  << " m, is_grasped=" << failed_grasp_state.is_grasped << "."
                  << std::endl;
      }
    }
    catch (const franka::Exception &e)
    {
      std::cerr << "[Follower Gripper] grasped exception:" << e.what() << std::endl;
    }
    return false;
  }

  // hold grasp
  if (grasp_flag && target_width < release_threshold)
  {
    return false;
  }

  // release
  if (grasp_flag && target_width >= release_threshold)
  {
    gripper.stop();

    const double max_width = gripper.readOnce().max_width;
    if (gripper.move(max_width, 0.1))
    {
      grasp_flag = false;
      std::cout << "[Follower Gripper] Released object at maximum width "
                << max_width << " m." << std::endl;
    }
    else
    {
      std::cerr << "[Follower Gripper] Failed to open for object release."
                << std::endl;
    }
    return false;
  }

  // exit gripper thread
  if (ever_grasped && !grasp_flag && target_width >= exit_threshold)
  {
    return true;
  }

  return false;
}

//=================Camera thread============================================================================

void rgbd_camera_thread_func(
    int camera_index, std::atomic<bool> &running, const std::string &out_dir,
    const std::string &camera_serial, std::atomic<int> &ready_count,
    std::atomic<bool> &capture_failed, std::atomic<uint64_t> &committed_frames)
{
  const int width = 640;
  const int height = 480;
  const int fps = 30;
  const std::string stem = out_dir + "/cam" + std::to_string(camera_index);
  uint64_t written_frames = 0;

  try
  {
    RealSenseCam1 camera(width, height, fps, camera_serial);
    cv::VideoWriter rgb_writer(
        stem + ".mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'),
        fps, cv::Size(width, height));
    const std::string depth_path = stem + "_depth.z16.zst";
    std::ofstream depth_stream(depth_path, std::ios::binary);
    std::ofstream timestamps(stem + "_timestamps.csv");
    if (!rgb_writer.isOpened() || !depth_stream || !timestamps)
    {
      throw std::runtime_error("could not open RGB-D output files for " + stem);
    }

    const rs2_intrinsics color = camera.colorIntrinsics();
    const rs2_intrinsics depth = camera.depthIntrinsics();
    const rs2_extrinsics depth_to_color = camera.depthToColorExtrinsics();
    std::array<float, 5> color_coefficients{};
    std::array<float, 5> depth_coefficients{};
    std::array<float, 9> depth_to_color_rotation{};
    std::array<float, 3> depth_to_color_translation{};
    std::copy_n(color.coeffs, color_coefficients.size(), color_coefficients.begin());
    std::copy_n(depth.coeffs, depth_coefficients.size(), depth_coefficients.begin());
    std::copy_n(
        depth_to_color.rotation, depth_to_color_rotation.size(),
        depth_to_color_rotation.begin());
    std::copy_n(
        depth_to_color.translation, depth_to_color_translation.size(),
        depth_to_color_translation.begin());
    json metadata = {
        {"format", "threading-realsense-rgbd-v1"},
        {"serial", camera.serial()},
        {"width", width},
        {"height", height},
        {"fps", fps},
        {"depth_scale_m", camera.depthScaleMeters()},
        {"depth_aligned_to", "color"},
        {"depth_storage", "independent zstd frames of row-major uint16 little-endian z16"},
        {"depth_file", fs::path(depth_path).filename().string()},
        {"depth_compression", "zstd"},
        {"depth_compression_level", 1},
        {"color_intrinsics",
         {{"width", color.width}, {"height", color.height},
          {"fx", color.fx}, {"fy", color.fy},
          {"ppx", color.ppx}, {"ppy", color.ppy},
          {"model", static_cast<int>(color.model)},
          {"coeffs", color_coefficients}}},
        {"native_depth_intrinsics",
         {{"width", depth.width}, {"height", depth.height},
          {"fx", depth.fx}, {"fy", depth.fy},
          {"ppx", depth.ppx}, {"ppy", depth.ppy},
          {"model", static_cast<int>(depth.model)},
          {"coeffs", depth_coefficients}}},
        {"native_depth_to_color",
         {{"rotation", depth_to_color_rotation},
          {"translation_m", depth_to_color_translation}}}};
    {
      std::ofstream metadata_stream(stem + "_metadata.json");
      metadata_stream << metadata.dump(2) << std::endl;
    }

    timestamps << "frame_index,color_frame_number,depth_frame_number,"
                  "color_sensor_timestamp_ms,depth_sensor_timestamp_ms,"
                  "host_steady_timestamp_ns,depth_offset_bytes,"
                  "depth_compressed_bytes\n";
    ready_count.fetch_add(1);

    const size_t raw_depth_bytes =
        static_cast<size_t>(width) * height * sizeof(uint16_t);
    std::vector<uint8_t> compressed_depth(ZSTD_compressBound(raw_depth_bytes));

    while (running && g_camera_episode_running.load())
    {
      RealSenseRgbdFrame frame;
      if (!camera.grabRgbd(frame))
      {
        throw std::runtime_error("lost RGB-D frame from camera " + camera_serial);
      }
      if (!g_record_active.load())
      {
        continue;
      }
      if (frame.host_timestamp_ns < g_record_start_time_ns.load())
      {
        continue;
      }
      if (frame.color_bgr.empty() || frame.depth_z16_aligned_to_color.empty() ||
          frame.depth_z16_aligned_to_color.type() != CV_16UC1)
      {
        throw std::runtime_error("invalid aligned RGB-D frame from " + camera_serial);
      }

      cv::Mat contiguous_depth = frame.depth_z16_aligned_to_color;
      if (!contiguous_depth.isContinuous())
      {
        contiguous_depth = frame.depth_z16_aligned_to_color.clone();
      }
      const size_t compressed_bytes = ZSTD_compress(
          compressed_depth.data(), compressed_depth.size(),
          contiguous_depth.ptr<uint16_t>(), raw_depth_bytes, 1);
      if (ZSTD_isError(compressed_bytes))
      {
        throw std::runtime_error(
            "zstd depth compression failed for " + camera_serial + ": " +
            ZSTD_getErrorName(compressed_bytes));
      }

      const std::streampos depth_offset = depth_stream.tellp();
      if (depth_offset < 0)
      {
        throw std::runtime_error("failed getting depth offset for " + camera_serial);
      }
      rgb_writer.write(frame.color_bgr);
      depth_stream.write(
          reinterpret_cast<const char *>(compressed_depth.data()),
          static_cast<std::streamsize>(compressed_bytes));
      if (!depth_stream)
      {
        throw std::runtime_error("failed writing depth data for " + camera_serial);
      }
      timestamps << written_frames << ',' << frame.color_frame_number << ','
                 << frame.depth_frame_number << ',' << frame.color_timestamp_ms
                 << ',' << frame.depth_timestamp_ms << ','
                 << frame.host_timestamp_ns << ','
                 << static_cast<uint64_t>(static_cast<std::streamoff>(depth_offset)) << ','
                 << compressed_bytes << '\n';
      if (!timestamps)
      {
        throw std::runtime_error("failed writing timestamps for " + camera_serial);
      }
      ++written_frames;
      committed_frames.store(written_frames, std::memory_order_release);
      if (written_frames % 30 == 0)
      {
        depth_stream.flush();
        timestamps.flush();
      }
    }

    depth_stream.flush();
    timestamps.flush();
    rgb_writer.release();
    std::cout << "[RGB-D cam" << camera_index << "] saved " << written_frames
              << " frames from " << camera_serial << std::endl;
  }
  catch (const std::exception &error)
  {
    capture_failed.store(true);
    running = false;
    std::cerr << "[RGB-D cam" << camera_index << "] " << error.what()
              << "; preserved " << written_frames << " committed frames"
              << std::endl;
  }
}
//=======================================================================================================

// Driver code
void udpwithremote_send(send_data &Data2send, std::atomic<bool> &running)
{
  uint16_t PORTSend = 5001;

  int sockfd;

  struct sockaddr_in servaddr, cliaddr;

  // Creating socket file descriptor
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  memset(&servaddr, 0, sizeof(servaddr));
  memset(&cliaddr, 0, sizeof(cliaddr));

  // Filling server information
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(PORTSend);
  // servaddr.sin_addr.s_addr = INADDR_ANY;
  servaddr.sin_addr.s_addr = inet_addr(IP_remote); // 10.162.15.208 10.162.15.234

  // const int NoDatatoSend = 29;
  const int NoDatatoSend = 41;

  while (running)
  {
    std::array<double, NoDatatoSend> msg2send;
    {
      std::unique_lock<std::mutex> lck_send(Data2send.mutex);
      cv_send.wait(lck_send, []
                   { return send_allowed; });

      msg2send = {
          Data2send.pandatime, Data2send.q_local_delta[0], Data2send.q_local_delta[1],
          Data2send.q_local_delta[2], Data2send.q_local_delta[3], Data2send.q_local_delta[4],
          Data2send.q_local_delta[5], Data2send.q_local_delta[6], Data2send.dq_local[0],
          Data2send.dq_local[1], Data2send.dq_local[2], Data2send.dq_local[3],
          Data2send.dq_local[4], Data2send.dq_local[5], Data2send.dq_local[6],
          Data2send.tau_local[0], Data2send.tau_local[1], Data2send.tau_local[2],
          Data2send.tau_local[3], Data2send.tau_local[4], Data2send.tau_local[5],
          Data2send.tau_local[6], Data2send.f_local[0], Data2send.f_local[1],
          Data2send.f_local[2], Data2send.f_local[3], Data2send.f_local[4],
          Data2send.f_local[5], Data2send.energy, Data2send.stop_code,
          Data2send.gripper_width.load(std::memory_order_acquire),
          Data2send.teleop_active, Data2send.episode_phase,
          Data2send.q_local_absolute[0], Data2send.q_local_absolute[1],
          Data2send.q_local_absolute[2], Data2send.q_local_absolute[3],
          Data2send.q_local_absolute[4], Data2send.q_local_absolute[5],
          Data2send.q_local_absolute[6], Data2send.robot_state_valid};
      send_allowed = false;
    }

    // Never hold Data2send.mutex across a potentially blocking system call.
    sendto(sockfd, msg2send.data(), sizeof(msg2send), MSG_CONFIRM, (const struct sockaddr *)&servaddr,
           sizeof(servaddr));
  }

  close(sockfd);
}

void udpwithremote_recv(recv_data &Data2Recv, std::atomic<bool> &running)
{
  uint16_t PORTRECV = 5001;

  int sockfd;

  struct sockaddr_in servaddr, cliaddr;

  // Creating socket file descriptor
  if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
  {
    perror("socket creation failed");
    exit(EXIT_FAILURE);
  }

  memset(&servaddr, 0, sizeof(servaddr));
  memset(&cliaddr, 0, sizeof(cliaddr));

  // Filling server information RECV
  cliaddr.sin_family = AF_INET; // IPv4
  cliaddr.sin_addr.s_addr = INADDR_ANY;
  cliaddr.sin_port = htons(PORTRECV);

  // Bind the socket with the server address
  if (bind(sockfd, (const struct sockaddr *)&cliaddr, sizeof(cliaddr)) < 0)
  {
    std::cout << "bind failed" << std::endl;
    exit(EXIT_FAILURE);
  }

  // int n, n_rev;
  socklen_t len;
  len = sizeof(cliaddr); // len is value/resuslt

  // const int NoDatatoRecv = 29; // 3*7+6+1+1
  const int NoDatatoRecv = 41;

  double msg2recv[NoDatatoRecv];
  uint64_t recv_count = 0;
  bool first_packet_logged = false;

  while (running)
  {
    len = sizeof(cliaddr);
    ssize_t n_recv = recvfrom(sockfd, msg2recv, sizeof(msg2recv), MSG_WAITALL,
                              (struct sockaddr *)&cliaddr, &len);
    if (n_recv < 0)
    {
      perror("recvfrom failed");
      continue;
    }
    if (static_cast<size_t>(n_recv) != sizeof(msg2recv))
    {
      std::cerr << "[UDP recv] unexpected packet size: " << n_recv
                << " bytes, expected " << sizeof(msg2recv) << " bytes"
                << std::endl;
      continue;
    }

    {
      std::lock_guard<std::mutex> recv_lock(Data2Recv.mutex);
      Data2Recv.remotetime = msg2recv[0];

      Data2Recv.q_remote_delta[0] = msg2recv[1];
      Data2Recv.q_remote_delta[1] = msg2recv[2];
      Data2Recv.q_remote_delta[2] = msg2recv[3];
      Data2Recv.q_remote_delta[3] = msg2recv[4];
      Data2Recv.q_remote_delta[4] = msg2recv[5];
      Data2Recv.q_remote_delta[5] = msg2recv[6];
      Data2Recv.q_remote_delta[6] = msg2recv[7];

      Data2Recv.dq_remote[0] = msg2recv[8];
      Data2Recv.dq_remote[1] = msg2recv[9];
      Data2Recv.dq_remote[2] = msg2recv[10];
      Data2Recv.dq_remote[3] = msg2recv[11];
      Data2Recv.dq_remote[4] = msg2recv[12];
      Data2Recv.dq_remote[5] = msg2recv[13];
      Data2Recv.dq_remote[6] = msg2recv[14];

      Data2Recv.tau_remote[0] = msg2recv[15];
      Data2Recv.tau_remote[1] = msg2recv[16];
      Data2Recv.tau_remote[2] = msg2recv[17];
      Data2Recv.tau_remote[3] = msg2recv[18];
      Data2Recv.tau_remote[4] = msg2recv[19];
      Data2Recv.tau_remote[5] = msg2recv[20];
      Data2Recv.tau_remote[6] = msg2recv[21];

      Data2Recv.f_remote[0] = msg2recv[22];
      Data2Recv.f_remote[1] = msg2recv[23];
      Data2Recv.f_remote[2] = msg2recv[24];
      Data2Recv.f_remote[3] = msg2recv[25];
      Data2Recv.f_remote[4] = msg2recv[26];
      Data2Recv.f_remote[5] = msg2recv[27];

      Data2Recv.energy = msg2recv[28];

      Data2Recv.stop_code = msg2recv[29];
      if (Data2Recv.stop_code > 0.5)
      {
        g_stop_requested.store(true);
      }

      Data2Recv.gripper_width = msg2recv[30];
      Data2Recv.teleop_active = msg2recv[31];
      Data2Recv.episode_phase = msg2recv[32];
      for (int i = 0; i < 7; ++i)
      {
        Data2Recv.q_remote_absolute[i] = msg2recv[33 + i];
      }
      Data2Recv.robot_state_valid = msg2recv[40];
      Data2Recv.last_receive_time_ns.store(steady_time_ns(), std::memory_order_release);
      Data2Recv.has_received.store(true);

      recv_count++;
      if (!first_packet_logged || recv_count % 1000 == 0)
      {
        first_packet_logged = true;
        double q_delta_norm = 0.0;
        double dq_norm = 0.0;
        for (int i = 0; i < 7; i++)
        {
          q_delta_norm += Data2Recv.q_remote_delta[i] * Data2Recv.q_remote_delta[i];
          dq_norm += Data2Recv.dq_remote[i] * Data2Recv.dq_remote[i];
        }
        std::cout << "[UDP recv] packets=" << recv_count
                  << " from=" << inet_ntoa(cliaddr.sin_addr)
                  << " q_delta_norm=" << std::sqrt(q_delta_norm)
                  << " dq_norm=" << std::sqrt(dq_norm)
                  << " grip=" << Data2Recv.gripper_width
                  << " teleop_active=" << Data2Recv.teleop_active
                  << " episode_phase=" << Data2Recv.episode_phase
                  << " stop=" << Data2Recv.stop_code
                  << std::endl;
      }
    }
  }
  close(sockfd);
}
