#include "hmi_node1.h"

HmiNode::HmiNode(QObject *parent):
    QObject(parent),
    Node("hmi_node")
{

    RCLCPP_INFO(this->get_logger(), "HMI node running!");

    transform_publisher = this->create_publisher<stewart_platform_interfaces::msg::TransformsPosVel>(
        "dof_ref", 1);
    
    RCLCPP_INFO(this->get_logger(), "Publishing transformations (DOF) ref on dof_ref topic");

    timer_ = this->create_wall_timer(
        10ms, std::bind(&HmiNode::timer_callback, this));
}

void HmiNode::timer_callback()
{
    static std::vector<std::vector<float>> output(2, std::vector<float>(6));

    sample_sine(output);

    auto msg_ = stewart_platform_interfaces::msg::TransformsPosVel();
    
    msg_.pose.x = output[0][0];
    msg_.pose.y = output[0][1];
    msg_.pose.z = output[0][2];
    
    msg_.twist.x = output[1][0];
    msg_.twist.y = output[1][1];
    msg_.twist.z = output[1][2];

    msg_.pose.roll  = output[0][3];
    msg_.pose.pitch = output[0][4];
    msg_.pose.yaw   = output[0][5];

    msg_.twist.roll  = output[1][3];
    msg_.twist.pitch = output[1][4];
    msg_.twist.yaw   = output[1][5];

    transform_publisher->publish(msg_);
    
    this->t += 0.01;
}

void HmiNode::sample_sine(std::vector<std::vector<float>>& dof)
{
    static const uint8_t POS = 0;
    static const uint8_t VEL = 1;

    dof[POS][0] = surge_A * sin(2.*PI * trans_freq * t + surge_offset) + surge_bias;
    dof[POS][1] = sway_A * sin(2.*PI * trans_freq * t + sway_offset) + sway_bias;
    dof[POS][2] = heave_A * sin(2.*PI * trans_freq * t + heave_offset) + heave_bias;

    dof[POS][3] = roll_A * (PI/180.) * sin(2.*PI * rot_freq * t + roll_offset) + roll_bias;
    dof[POS][4] = pitch_A * (PI/180.) * sin(2.*PI * rot_freq * t + pitch_offset) + pitch_bias;
    dof[POS][5] = yaw_A * (PI/180.) * sin(2.*PI * rot_freq * t + yaw_offset) + yaw_bias;


    dof[VEL][0] = surge_A * 2.*PI * trans_freq * cos(2*PI * trans_freq * t + surge_offset);
    dof[VEL][1] = sway_A * 2.*PI * trans_freq * cos(2*PI * trans_freq * t + sway_offset);
    dof[VEL][2] = heave_A * 2.*PI * trans_freq * cos(2*PI * trans_freq * t + heave_offset);

    dof[VEL][3] = roll_A * (PI/180.) * 2.*PI * rot_freq * cos(2*PI * rot_freq * t + roll_offset);
    dof[VEL][4] = pitch_A * (PI/180.) * 2.*PI * rot_freq * cos(2*PI * rot_freq * t + pitch_offset);
    dof[VEL][5] = yaw_A * (PI/180.) * 2.*PI * rot_freq * cos(2*PI * rot_freq * t + yaw_offset);

}

void HmiNode::set_trans_freq(const float& trans_freq)
{
    this->trans_freq = trans_freq * 0.5;
    qDebug() << "Translative frequency changed: " << trans_freq * 0.5;
}

void HmiNode::set_rot_freq(const float& rot_freq)
{
    this->rot_freq = rot_freq * 0.5;
    qDebug() << "Rotational frequency changed: " << rot_freq * 0.5;
}

void HmiNode::set_surge_A(const float& surge_A)
{
    this->surge_A = surge_A * 0.03;
    qDebug() << "Surge amplitude changed: " << surge_A * 0.03;
}

void HmiNode::set_sway_A(const float& sway_A)
{
    this->sway_A = sway_A * 0.03;
    qDebug() << "Sway amplitude changed: " << sway_A * 0.03;
}

void HmiNode::set_heave_A(const float& heave_A)
{
    this->heave_A = heave_A * 0.03;
    qDebug() << "Heave amplitude changed: " << heave_A * 0.03;
}

void HmiNode::set_roll_A(const float& roll_A)
{
    this->roll_A = roll_A * 10.;
    qDebug() << "Roll amplitude changed: " << roll_A * 10.;
}

void HmiNode::set_pitch_A(const float& pitch_A)
{
    this->pitch_A = pitch_A * 10.;
    qDebug() << "Pitch amplitude changed: " << pitch_A * 10.;
}

void HmiNode::set_yaw_A(const float& yaw_A)
{
    this->yaw_A = yaw_A * 40.;
    qDebug() << "Yaw amplitude changed: " << yaw_A * 40.;
}