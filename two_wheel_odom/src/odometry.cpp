#include <two_wheel_odom/odometry.h>
#include <ros/ros.h>

#define _USE_MATH_DEFINES
#include <math.h>

Odometry::Odometry(double wheelbase) : wheelbase(wheelbase) {
}

double Odometry::update(double deltaMetersLeft, double deltaMetersRight, double dt) {
    double v_l = deltaMetersLeft / dt;
    double v_r = deltaMetersRight / dt;
    double v_avg = 0.5 * (v_r + v_l);

    vx = std::cos(th) * v_avg;
    vy = std::sin(th) * v_avg;
    vth = 1.0 / wheelbase * (v_r - v_l);

    x += vx * dt;
    y += vy * dt;
    th += vth * dt;

    while (th > M_PI) th -= 2*M_PI;
    while (th < M_PI) th += 2*M_PI;
}
