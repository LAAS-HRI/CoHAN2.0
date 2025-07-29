/*******************************************************************************
 * Example of how to use the generic parameter utility in any ROS2 node
 *********************************************************************************/

#include <agent_path_prediction/parameter_utils.hpp>
#include <rclcpp/rclcpp.hpp>

class ExampleNode : public rclcpp::Node {
 public:
  ExampleNode() : Node("example_node"), param_helper_(shared_from_this()) { setupParameters(); }

 private:
  void setupParameters() {
    // Declare different types of parameters
    param_helper_.declareFloatParam("max_velocity", 1.0, 0.1, 5.0, "Maximum velocity of the robot");
    param_helper_.declareFloatParam("goal_tolerance", 0.2, 0.01, 1.0, "Goal tolerance for navigation");
    param_helper_.declareIntParam("max_iterations", 100, 1, 1000, "Maximum number of planning iterations");
    param_helper_.declareBoolParam("enable_safety", true, "Enable safety features");
    param_helper_.declareStringParam("robot_frame", "base_link", "Robot base frame");

    // Setup parameter callback with custom validation
    param_helper_.setupParameterCallback([this](const std::vector<rclcpp::Parameter> &params) -> bool {
      for (const auto &param : params) {
        const std::string &name = param.get_name();

        // Custom validation logic
        if (name == "max_velocity" && param.as_double() > 3.0) {
          RCLCPP_WARN(get_logger(), "Max velocity %.2f is quite high!", param.as_double());
        }

        if (name == "robot_frame" && param.as_string().empty()) {
          RCLCPP_ERROR(get_logger(), "Robot frame cannot be empty!");
          return false;
        }

        // Update internal variables
        updateInternalParams();
      }
      return true;
    });
  }

  void updateInternalParams() {
    max_velocity_ = param_helper_.getParam<double>("max_velocity", 1.0);
    goal_tolerance_ = param_helper_.getParam<double>("goal_tolerance", 0.2);
    max_iterations_ = param_helper_.getParam<int>("max_iterations", 100);
    enable_safety_ = param_helper_.getParam<bool>("enable_safety", true);
    robot_frame_ = param_helper_.getParam<std::string>("robot_frame", "base_link");
  }

  // Parameter helper instance
  parameter_utils::ParameterHelper param_helper_;

  // Internal parameter variables
  double max_velocity_;
  double goal_tolerance_;
  int max_iterations_;
  bool enable_safety_;
  std::string robot_frame_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ExampleNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
