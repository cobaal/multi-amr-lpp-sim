#include <rclcpp/rclcpp.hpp>
#include <turtlesim/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>
#include <cmath>
#include <nlohmann/json.hpp>
#include <fstream>

using json = nlohmann::json;

turtlesim::msg::Pose g_pose;

// 설정: Lookahead 거리 및 목표 인덱스
double lookahead_distance = 1.0;
size_t current_goal_index = 0;

// 시작지점들
turtlesim::msg::Pose beginning_point;

std::vector<turtlesim::msg::Pose> goals;

rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub;

void poseCallback(const turtlesim::msg::Pose::SharedPtr msg)
{
  g_pose = *msg;

  // 터미널에 값을 출력
  // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), 
  //             "x: %.6f\ny: %.6f\ntheta: %.6f\nlinear_velocity: %.6f\nangular_velocity: %.6f\n---",
  //             msg->x, msg->y, msg->theta, msg->linear_velocity, msg->angular_velocity);

}

// 함수: 두 점 사이의 유클리드 거리 계산
double calculateDistance(double x1, double y1, double x2, double y2)
{
  return std::sqrt(std::pow(x2 - x1, 2) + std::pow(y2 - y1, 2));
}

double normalizeAngle(double angle)
{
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Pure Pursuit 알고리즘을 사용해 제어 명령을 계산하는 함수
geometry_msgs::msg::Twist purePursuitControl(   const turtlesim::msg::Pose& current_pose, 
                                                const turtlesim::msg::Pose& beginning_point,
                                                const turtlesim::msg::Pose& goal, 
                                                double lookahead_distance)
{
  geometry_msgs::msg::Twist twist_msg;

  // 시작 지점으로부터의 거리
  double distance_to_beginning = calculateDistance(current_pose.x, current_pose.y, beginning_point.x, beginning_point.y);

  // 목표 지점까지의 거리
  double distance_to_goal = calculateDistance(current_pose.x, current_pose.y, goal.x, goal.y);

  // 로봇과 목표 간의 각도 차이 계산
  double angle_to_goal = std::atan2(goal.y - current_pose.y, goal.x - current_pose.x);
  // double angle_diff = angle_to_goal - current_pose.theta;
  double angle_diff = normalizeAngle(angle_to_goal - current_pose.theta); // 각도 차이 정규화


  // Pure Pursuit에 따라 속도 결정
  double k_linear = 2.0;
  double k_angular = 2.0;

  // if (distance_to_beginning < lookahead_distance) {
  //   // 시작할 때 서서히 속도 높이기
  //   twist_msg.linear.x = k_linear * lookahead_distance * (distance_to_beginning + 0.1);
  //   twist_msg.angular.z = k_angular * angle_diff;

  // } else if (distance_to_goal > lookahead_distance) {
  //   twist_msg.linear.x = k_linear * lookahead_distance; // 일정한 선속도를 사용
  //   twist_msg.angular.z = k_angular * angle_diff;       // 각도 차이에 따라 회전 속도를 설정

  // } else {
  //   // 목표에 가까워졌을 때 속도 줄이기
  //   twist_msg.linear.x = k_linear * lookahead_distance * distance_to_goal;
  //   // twist_msg.angular.z = 0.0;
  //   twist_msg.angular.z = k_angular * angle_diff;       // 각도 차이에 따라 회전 속도를 설정
  // }

  if (distance_to_goal > lookahead_distance) {
    // 목표에 가까워졌을 때 속도 줄이기
    twist_msg.linear.x = k_linear * lookahead_distance * distance_to_goal;
    // twist_msg.angular.z = 0.0;
    twist_msg.angular.z = k_angular * angle_diff;       // 각도 차이에 따라 회전 속도를 설정

  } else if (distance_to_beginning < lookahead_distance) {
    // 시작할 때 서서히 속도 높이기
    twist_msg.linear.x = k_linear * lookahead_distance * (distance_to_beginning + 0.1);
    twist_msg.angular.z = k_angular * angle_diff;

  } else {
    twist_msg.linear.x = k_linear * lookahead_distance; // 일정한 선속도를 사용
    twist_msg.angular.z = k_angular * angle_diff;       // 각도 차이에 따라 회전 속도를 설정
  }

  return twist_msg;
}

void controlCallback()
{
  if (current_goal_index < goals.size())
  {
    auto goal = goals[current_goal_index];
    // std::cout << current_goal_index << std::endl;

    // Pure Pursuit 제어 함수 호출
    geometry_msgs::msg::Twist twist_msg = purePursuitControl(g_pose, beginning_point, goal, lookahead_distance);

    // 속도 명령 퍼블리시
    twist_pub->publish(twist_msg);

    // 목표에 도달했는지 확인
    double distance_to_goal = calculateDistance(g_pose.x, g_pose.y, goal.x, goal.y);
    // std::cout << distance_to_goal << std::endl;

    if (distance_to_goal < 0.15)
    {
      beginning_point = goal;
      current_goal_index++;
      if (current_goal_index >= goals.size())
      {
        current_goal_index = 0; // 마지막 목표에 도달하면 다시 첫 번째 목표로 이동
        // break;
      }
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // std::cout << argc << std::endl;

  std::string node_name = "pure_pursuit_controller";
  std::string turtle_name = "turtle";
  std::string pose_name = "pose";
  std::string cmd_vel_name = "cmd_vel";

  if (argc == 2) {
    node_name += std::string(argv[1]);
    turtle_name = "turtle" + std::string(argv[1]);
    pose_name = turtle_name + "/" + pose_name;
    cmd_vel_name = turtle_name + "/" + cmd_vel_name;
  } 

  auto nh = rclcpp::Node::make_shared(node_name);
  
  // 구독자 및 퍼블리셔 생성
  rclcpp::QoS qos_test = rclcpp::QoS(rclcpp::KeepAll());
  auto pose_sub = nh->create_subscription<turtlesim::msg::Pose>(pose_name, qos_test, poseCallback);
  // auto twist_pub = nh->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_name, qos_test);
  twist_pub = nh->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_name, qos_test);

  // JSON 파일 경로
  std::string json_file_path = std::getenv("WAYPOINTS_PATH");

  // JSON 파일 읽기
  std::ifstream file(json_file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(nh->get_logger(), "Failed to open JSON file: %s", json_file_path.c_str());
    return 1;
  }

  // JSON 파일 파싱
  json waypoints_json;
  file >> waypoints_json;

  // // 목표 지점 설정
  // std::vector<turtlesim::msg::Pose> goals;
  // for (const auto& waypoint : waypoints_json) {
  //   turtlesim::msg::Pose pose;
  //   pose.x = waypoint["x"];
  //   pose.y = waypoint["y"];
  //   pose.theta = waypoint["theta"];
  //   goals.push_back(pose);
  // }

  // 목표 지점 설정
  if (waypoints_json.contains(turtle_name) && waypoints_json[turtle_name].is_array()) {
    for (const auto& waypoint : waypoints_json[turtle_name]) {
      turtlesim::msg::Pose pose;
      pose.x = waypoint["x"];
      pose.y = waypoint["y"];
      pose.theta = waypoint["theta"];
      goals.push_back(pose);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "JSON key not found or not an array in JSON data.");
    throw std::runtime_error("JSON key not found or not an array in JSON data.");
  }


  // // 목표 지점 설정
  // std::vector<turtlesim::msg::Pose> goals;

  // turtlesim::msg::Pose pose1;
  // pose1.x = 6.565789699554443;
  // pose1.y = 6.565789699554443;
  // pose1.theta = 0.0;
  // goals.push_back(pose1);

  // turtlesim::msg::Pose pose2;
  // pose2.x = 6.565789699554443;
  // pose2.y = 1.0;
  // pose2.theta = 0.0;
  // goals.push_back(pose2);

  // turtlesim::msg::Pose pose3;
  // pose3.x = 6.565789699554443;
  // pose3.y = 10.0;
  // pose3.theta = 0.0;
  // goals.push_back(pose3);
  
  // turtlesim::msg::Pose pose4;
  // pose4.x = 1.0;
  // pose4.y = 10.0;
  // pose4.theta = 0.0;
  // goals.push_back(pose4);

  // turtlesim::msg::Pose pose5;
  // pose5.x = 1.0;
  // pose5.y = 6.565789699554443;
  // pose5.theta = 0.0;
  // goals.push_back(pose5);

  // // 설정: Lookahead 거리 및 목표 인덱스
  // double lookahead_distance = 1.0;
  // size_t current_goal_index = 0;

  // // 시작지점들
  // turtlesim::msg::Pose beginning_point;
  beginning_point.x = 0.0;
  beginning_point.y = 0.0;
  beginning_point.theta = 0.0;

  // rclcpp::Rate rate(10.0); // 10 Hz

  // while (rclcpp::ok())
  // {
  //   rclcpp::spin_some(nh);

  //   if (current_goal_index < goals.size())
  //   {
  //     auto goal = goals[current_goal_index];
  //     // std::cout << current_goal_index << std::endl;

  //     // Pure Pursuit 제어 함수 호출
  //     geometry_msgs::msg::Twist twist_msg = purePursuitControl(g_pose, beginning_point, goal, lookahead_distance);

  //     // 속도 명령 퍼블리시
  //     twist_pub->publish(twist_msg);

  //     // 목표에 도달했는지 확인
  //     double distance_to_goal = calculateDistance(g_pose.x, g_pose.y, goal.x, goal.y);
  //     // std::cout << distance_to_goal << std::endl;

  //     if (distance_to_goal < 0.15)
  //     {
  //       beginning_point = goal;
  //       current_goal_index++;
  //       if (current_goal_index >= goals.size())
  //       {
  //         current_goal_index = 0; // 마지막 목표에 도달하면 다시 첫 번째 목표로 이동
  //         // break;
  //       }
  //     }
  //   }

  //   rate.sleep();
  // }

  auto mTimer = nh->create_wall_timer(std::chrono::milliseconds(100), controlCallback);
  rclcpp::spin(nh);

  rclcpp::shutdown();
  return 0;
}
