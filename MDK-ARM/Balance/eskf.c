#include "eskf.h"

#define ESKF_PI 3.14159265358979323846

static double wrap_angle(double a) {
    while (a > ESKF_PI) {
        a -= 2.0 * ESKF_PI;
    }
    while (a < -ESKF_PI) {
        a += 2.0 * ESKF_PI;
    }
    return a;
}

static void mat3_mul(const double A[3][3], const double B[3][3], double C[3][3]) {
    int i, j, k;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (k = 0; k < 3; ++k) {
                sum += A[i][k] * B[k][j];
            }
            C[i][j] = sum;
        }
    }
}

static void mat3_transpose(const double A[3][3], double AT[3][3]) {
    int i, j;
    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            AT[j][i] = A[i][j];
        }
    }
}

void eskf_init(eskf_t *filter) {
    int i;
    memset(filter, 0, sizeof(eskf_t));
    filter->nominal.x = 0.0;
    filter->nominal.y = 0.0;
    filter->nominal.yaw = 0.0;

    filter->P[0][0] = 0.1;
    filter->P[1][1] = 0.1;
    filter->P[2][2] = 1.0e-8; // 【修改 1】让 Yaw 的初始不确定性极小

    filter->R[0][0] = 1.0e-4; // 也可以换成动态 R 的初始值
    filter->R[1][1] = 1.0e-4;  

    filter->Q[0][0] = 1.0e-5;  
    filter->Q[1][1] = 1.0e-5;
    
    // 【修改 2】极其关键！停止累积 Yaw 的误差！
    filter->Q[2][2] = 0.0;
}

void eskf_predict(eskf_t *filter, double vx, double vy, double yaw_rad, double dt) {
    double c = cos(yaw_rad);
    double s = sin(yaw_rad);
    double dx = (vx * c - vy * s) * dt;
    double dy = (vx * s + vy * c) * dt;
    double F[3][3];
    double FP[3][3];
    double FT[3][3];
    double Pnew[3][3];
    int i, j;

    filter->nominal.x += dx;
    filter->nominal.y += dy;
    filter->nominal.yaw = wrap_angle(yaw_rad);

    F[0][0] = 1.0;
    F[0][1] = 0.0;
    F[0][2] = (-vx * s - vy * c) * dt;
    F[1][0] = 0.0;
    F[1][1] = 1.0;
    F[1][2] = (vx * c - vy * s) * dt;
    F[2][0] = 0.0;
    F[2][1] = 0.0;
    F[2][2] = 1.0;

    mat3_mul(F, filter->P, FP);
    mat3_transpose(F, FT);
    mat3_mul(FP, FT, Pnew);

    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            Pnew[i][j] += filter->Q[i][j];
        }
    }

    for (i = 0; i < 3; ++i) {
        for (j = 0; j < 3; ++j) {
            filter->P[i][j] = Pnew[i][j];
        }
    }
}

void eskf_update(eskf_t *filter, double meas_x, double meas_y, float vx, float vy) {
    // 1. 计算观测残差 (实际 UWB 坐标 - 预测坐标)
    double z0 = meas_x - filter->nominal.x;
    double z1 = meas_y - filter->nominal.y;

    // ==========================================
    // “防绑架”与“防飞点”逻辑
    // ==========================================
    static int kidnap_count = 0; // 连续异常帧计数器

    if (fabs(z0) > 1.0 || fabs(z1) > 1.0) {
        kidnap_count++;
        
        // 假设 UWB 更新频率约为 10Hz-20Hz。
        // 连续 20 帧（大概 1~2 秒）都相差 1.5 米以上，确诊被搬走！
        if (kidnap_count > 20) { 
            // 强制瞬移到新位置
            filter->nominal.x = meas_x;
            filter->nominal.y = meas_y;
            
            // 重置 X 和 Y 的协方差，重建信任
            filter->P[0][0] = 0.1;
            filter->P[1][1] = 0.1;
            
            kidnap_count = 0; // 计数器清零
            return; // 重置完毕，本帧结束
        }
        
            // 如果连续异常还没达到 20 帧，先当作 UWB 飞点处理：不听不信
            return; 
        } else {
            // 只要有一帧数据恢复到了 1.5 米以内，说明没被搬走，报警解除
            kidnap_count = 0; 
        }
    // ==========================================

    // ==========================================
    // 2：解决“运动晃动”与“静止抖动” (自适应 R 阵)
    // ==========================================

    // 计算当前车速
    double speed = sqrt(vx * vx + vy * vy);

    // 动态 R 阵：基础噪声 0.05，速度越快，R 越大（越不信任 UWB）
    // 假设速度为 0.5m/s 时，R 会变成 0.05 + 0.5 * 0.2 = 0.15
    double dynamic_R = 5.0e-2 + speed * 0.4; 

    // 如果速度特别慢（接近静止），保持一个较小的 R，让 UWB 修正静态漂移
    if (vx == 0.0f && vy == 0.0f)  {
        dynamic_R = ESKF_R_STATIC; // 静止时把 R 开大，过滤原地漂移
    }

    filter->R[0][0] = dynamic_R;
    filter->R[1][1] = dynamic_R;

    // ==========================================

    // 2. 计算新息协方差矩阵 S = H*P*H^T + R
    double s00 = filter->P[0][0] + filter->R[0][0];
    double s01 = filter->P[0][1] + filter->R[0][1]; // 实际上 R 的非对角线为 0
    double s10 = filter->P[1][0] + filter->R[1][0];
    double s11 = filter->P[1][1] + filter->R[1][1];

    // 3. 对 S 求逆 (2x2矩阵解析逆)
    double det = s00 * s11 - s01 * s10;
    if (fabs(det) < 1e-9) {
        return; // 防止除以 0 导致系统崩溃
    }

    double inv00 = s11 / det;
    double inv01 = -s01 / det;
    double inv10 = -s10 / det;
    double inv11 = s00 / det;

    // 4. 计算卡尔曼增益 K = P*H^T*S^-1
    double k00 = filter->P[0][0] * inv00 + filter->P[0][1] * inv10;
    double k01 = filter->P[0][0] * inv01 + filter->P[0][1] * inv11;
    double k10 = filter->P[1][0] * inv00 + filter->P[1][1] * inv10;
    double k11 = filter->P[1][0] * inv01 + filter->P[1][1] * inv11;

    // 【修改 1：物理阉割卡尔曼增益】
    // 强制将 Yaw 对 UWB 观测的增益设为 0，彻底切断交叉协方差的污染
    double k20 = 0.0;
    double k21 = 0.0;

    // 5. 计算误差状态并注入名义状态
    double dx = k00 * z0 + k01 * z1;
    double dy = k10 * z0 + k11 * z1;
    
    filter->nominal.x += dx;
    filter->nominal.y += dy;
    
    // 【修改 2：彻底禁止 UWB 触碰 Yaw】
    // 绝对不让 UWB 更新 Yaw，让 MPU6050 拥有 100% 的控制权！
    // 不再计算 dtheta，也不更新 filter->nominal.yaw

    // 6. 更新误差状态协方差 P = (I - K*H)*P
    {
        double I_KH[3][3];
        double Pnew[3][3];
        int i, j;

        I_KH[0][0] = 1.0 - k00;
        I_KH[0][1] = -k01;
        I_KH[0][2] = 0.0;
        I_KH[1][0] = -k10;
        I_KH[1][1] = 1.0 - k11;
        I_KH[1][2] = 0.0;
        I_KH[2][0] = -k20; // 这里其实就是 -0.0
        I_KH[2][1] = -k21; // 这里其实就是 -0.0
        I_KH[2][2] = 1.0;

        mat3_mul(I_KH, filter->P, Pnew);
        for (i = 0; i < 3; ++i) {
            for (j = 0; j < 3; ++j) {
                filter->P[i][j] = Pnew[i][j];
            }
        }
    }
}