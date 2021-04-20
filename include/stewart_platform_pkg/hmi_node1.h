#ifndef HMI_NODE_H
#define HMI_NODE_H

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QDebug>
#include <QQmlContext>
#include <QThread>

#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "stewart_platform_interfaces/msg/transforms_pos_vel.hpp"

#include <vector>
#include <stdio.h>
#include <math.h>

#define PI 3.141592654

using namespace std::chrono_literals;

class HmiNode : public QObject, public rclcpp::Node
{
    Q_OBJECT

    public:
        HmiNode(QObject* parent = nullptr);
        virtual ~HmiNode() {};

        void sample_sine(std::vector<std::vector<float>>& dof);

    public slots:
        void set_trans_freq(const float& trans_freq);
        void set_rot_freq(const float& rot_freq);
        void set_surge_A(const float& surge_A);
        void set_sway_A(const float& sway_A);
        void set_heave_A(const float& heave_A);
        void set_roll_A(const float& roll_A);
        void set_pitch_A(const float& pitch_A);
        void set_yaw_A(const float& yaw_A);

    private:
        void timer_callback();

        float t = 0.;

        float trans_freq = 0.;
        float rot_freq = 0.;

        float surge_A = 0.;
        float surge_bias = 0.;
        float surge_offset = 0.;

        float sway_A = 0.;
        float sway_bias = 0.;
        float sway_offset = -PI/2;

        float heave_A = 0.;
        float heave_bias = 0.04853;
        float heave_offset = 0.;

        float roll_A = 0.;
        float roll_bias = 0.;
        float roll_offset = 0.;

        float pitch_A = 0.;
        float pitch_bias = 0.;
        float pitch_offset = -PI/2;

        float yaw_A = 0.;
        float yaw_bias = 0.;
        float yaw_offset = 0.;

        rclcpp::Publisher<stewart_platform_interfaces::msg::TransformsPosVel>::SharedPtr transform_publisher;
        rclcpp::TimerBase::SharedPtr timer_;
};



#endif