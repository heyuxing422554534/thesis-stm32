#ifndef ESKF_H
#define ESKF_H

#include <math.h>
#include <string.h>

// 2D nominal state
typedef struct {
    double x;    // Position X
    double y;    // Position Y
    double yaw;  // Heading (rad)
} eskf_state_t;

// 2D ESKF core
typedef struct {
    eskf_state_t nominal;
    double P[3][3];  // Error covariance for [dx, dy, dtheta]
    double Q[3][3];  // Process noise covariance
    double R[2][2];  // Measurement noise covariance (UWB x,y)
} eskf_t;

// 速度死区：低于该值的编码器速度视为 0（防止静止时积分漂移）
// 原值 0.04 m/s；末端逼近最低速度约 0.03 m/s，必须低于它，否则末端移动时位置不更新
#define ESKF_VEL_DEADBAND   0.01f
// 静止时 UWB 观测噪声 R（原值 1.0；越小越信 UWB，静止时位置修正越快但抖动越大）
#define ESKF_R_STATIC       0.2

void eskf_init(eskf_t *filter);
void eskf_predict(eskf_t *filter, double vx, double vy, double yaw_rad, double dt);
void eskf_update(eskf_t *filter, double meas_x, double meas_y, float vx, float vy);

#endif // ESKF_H
