#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Pose.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/twist.hpp> // <--- AGREGADO: Para que reconozca Twist
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "uuv_dynamic_model.h"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class SubmarineDynamics : public System, 
                          public ISystemConfigure, 
                          public ISystemPreUpdate {
public:
    SubmarineDynamics() {
        if (!rclcpp::ok()) {
            rclcpp::init(0, nullptr);
        }
        ros_node_ = std::make_shared<rclcpp::Node>("gz_submarine_dynamics");
        
        subscription_ = ros_node_->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/forces", 10, std::bind(&SubmarineDynamics::OnForcesMsg, this, std::placeholders::_1));

        dynamics_pub_ = ros_node_->create_publisher<std_msgs::msg::Float64MultiArray>("/dynamics", 10);
        velocity_pub_ = ros_node_->create_publisher<geometry_msgs::msg::Twist>("/velocity", 10);
        pose_pub_ = ros_node_->create_publisher<geometry_msgs::msg::PoseStamped>("/pose", 10);
        
        std::fill(thruster_input_.begin(), thruster_input_.end(), 0.0);
    }

    void Configure(const Entity &_entity, const std::shared_ptr<const sdf::Element> &,
                  EntityComponentManager &, EventManager &) override {
        m_model = Model(_entity);
    }

    void PreUpdate(const UpdateInfo &_info, EntityComponentManager &_ecm) override {
        if (_info.paused) return;

        rclcpp::spin_some(ros_node_);

        double dt_sim = std::chrono::duration<double>(_info.dt).count();

        m_dynamicModel.setDt(dt_sim);
        m_dynamicModel.update(thruster_input_); 

        // IMPORTANTE: Primero obtenemos el estado, luego lo usamos
        auto state = m_dynamicModel.state;

        // 1. Publicar Matrices Dynamics (f y g) para el ASMC
        auto dyn_msg = std_msgs::msg::Float64MultiArray();
        for (int i = 0; i < 6; i++) {
            for (int j = 0; j < 6; j++) {
                dyn_msg.data.push_back(m_dynamicModel.g_x_(i, j));
            }
        }
        for (int i = 0; i < 6; i++) {
            dyn_msg.data.push_back(m_dynamicModel.f_x_(i));
        }
        dynamics_pub_->publish(dyn_msg);

        // 2. Publicar Velocidad (Twist)
        geometry_msgs::msg::Twist vel_msg;
        vel_msg.linear.x = state.nu(0);
        vel_msg.linear.y = state.nu(1);
        vel_msg.linear.z = state.nu(2);
        vel_msg.angular.x = state.nu(3);
        vel_msg.angular.y = state.nu(4);
        vel_msg.angular.z = state.nu(5);
        velocity_pub_->publish(vel_msg);

        // 3. Actualizar la Pose en Gazebo
        // math::Pose3d nextPose(
        //     state.eta(0), state.eta(1), state.eta(2),
        //     state.eta(3), state.eta(4), state.eta(5)
        // );

        math::Pose3d nextPose(
            state.eta(0),   // X_enu = X_ned (Norte = Este en este contexto simplificado)
            state.eta(1),  // Y_enu = -Y_ned (Este a Izquierda)
            state.eta(2),  // Z_enu = -Z_ned (Bajar en NED es negativo en Gazebo)
            state.eta(3),   // Roll
            state.eta(4),  // Pitch
            state.eta(5)   // Yaw
        );

        auto poseComp = _ecm.Component<components::Pose>(m_model.Entity());
        if (poseComp) {
            *poseComp = components::Pose(nextPose);
            _ecm.SetChanged(m_model.Entity(), components::Pose::typeId, ComponentState::OneTimeChange);
        }

        geometry_msgs::msg::PoseStamped ros_pose;
        ros_pose.header.stamp = ros_node_->get_clock()->now();
        ros_pose.header.frame_id = "world"; // O "odom" según tu configuración
        ros_pose.pose.position.x = nextPose.Pos().X();
        ros_pose.pose.position.y = nextPose.Pos().Y();
        ros_pose.pose.position.z = nextPose.Pos().Z();
        ros_pose.pose.orientation.x = nextPose.Rot().X();
        ros_pose.pose.orientation.y = nextPose.Rot().Y();
        ros_pose.pose.orientation.z = nextPose.Rot().Z();
        ros_pose.pose.orientation.w = nextPose.Rot().W();

        pose_pub_->publish(ros_pose);
    }

private:
    void OnForcesMsg(const std_msgs::msg::Float64MultiArray::SharedPtr _msg) {
        if (_msg->data.size() >= 6) {
            for(int i=0; i<6; ++i) thruster_input_[i] = _msg->data[i];
        }
    }

    Model m_model;
    std::shared_ptr<rclcpp::Node> ros_node_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr dynamics_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    UUVDynamicModel m_dynamicModel;
    std::array<double, 6> thruster_input_;
};

IGNITION_ADD_PLUGIN(SubmarineDynamics,
                    ignition::gazebo::System,
                    SubmarineDynamics::ISystemConfigure,
                    SubmarineDynamics::ISystemPreUpdate)