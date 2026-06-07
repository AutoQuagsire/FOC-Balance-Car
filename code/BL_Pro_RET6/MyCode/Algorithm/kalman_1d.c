#include "kalman_1d.h"
#include <math.h>





static float R_normal = 3e-3f;

void Kalman1D_Init(Kalman1D_t *kf, float init_angle_rad, float init_bias_radps)
{
    if (kf == 0) {
        return;
    }
    
    //1.初始化状态和输入
    kf->angle_rad = init_angle_rad;
    kf->angle_degree = init_angle_rad * 180.0f / 3.14159f;
    kf->bias_radps = init_bias_radps;
    kf->rate_radps = 0.0f;              //去零偏后的角速度

    
    //2.估计误差协方差矩阵，Update中会在先验/后验之间更新
    kf->Pt[0][0] = 0.01f; // 角度估计初始误差较大
    kf->Pt[0][1] = 0.0f;
    kf->Pt[1][0] = 0.0f;
    kf->Pt[1][1] = 0.01f; 

    //3.初始化Q、R参数
    kf->Q_angle = 1e-5f;    //角度预测过程噪声   角速度瞬时测量噪声在这个模型里主要折算进 Q_angle。
    kf->Q_bias = 1e-8f;     //陀螺仪偏移噪声
    kf->R_measure = R_normal;  //加速度计测量噪声

    //4.初始化调试量
    kf->K0 = 0.0f;
    kf->K1 = 0.0f;
    kf->residual = 0.0f;

}





float Kalman1D_Update(Kalman1D_t *kf, IMU_PhysData_t IMU_PhysData, float dt)
{
    if (kf == 0 || dt <= 0.0f) {
    return 0.0f;
    }
    kf->accel_pitch_rad = -atan2f(IMU_PhysData.ax_mps2,
                             sqrtf(IMU_PhysData.ay_mps2 * IMU_PhysData.ay_mps2 +
                                   IMU_PhysData.az_mps2 * IMU_PhysData.az_mps2));
    kf->acc_norm = sqrtf(IMU_PhysData.ax_mps2 * IMU_PhysData.ax_mps2 +
                        IMU_PhysData.ay_mps2 * IMU_PhysData.ay_mps2 +
                        IMU_PhysData.az_mps2 * IMU_PhysData.az_mps2);
    //1.建立状态方程
    kf->rate_radps = IMU_PhysData.gy_radps - kf->bias_radps; // 去零偏后的角速度
    kf->angle_rad += kf->rate_radps * dt;                   //更新先验预测角度


    // Q_angle / Q_bias are process noise intensity per second.
    // Q_angle_d / Q_bias_d are discrete process noise covariance per update.
    float Q_angle_d = kf->Q_angle * dt;
    float Q_bias_d  = kf->Q_bias  * dt;

    //2.先验协方差预测
    kf->Pt[0][0] = kf->Pt[0][0] - kf->Pt[0][1] * dt - kf->Pt[1][0] * dt + kf->Pt[1][1] * dt * dt +  Q_angle_d;
    kf->Pt[0][1] = kf->Pt[0][1] - kf->Pt[1][1] * dt;
    kf->Pt[1][0] = kf->Pt[1][0] - kf->Pt[1][1] * dt;
    kf->Pt[1][1] = kf->Pt[1][1] + Q_bias_d;

    //3.残差公式
    //残差的维度由测量值 z 决定，不由状态量 x 决定,所以这里没有陀螺仪bias相关的残差
    float yt = kf->accel_pitch_rad - 1 * kf->angle_rad;
    kf->residual = yt;


    float acc_err = fabsf(kf->acc_norm - 9.80665f);
    float res_abs = fabsf(kf->residual);

    if (acc_err > 2.0f || res_abs > 0.70f) {
        kf->R_measure = 0.20f;      // 严重污染，几乎不信 accel
    } else if (acc_err > 1.0f || res_abs > 0.35f) {
        kf->R_measure = 0.02f;      // 中等污染，少信 accel
    } else {
        kf->R_measure = 0.003f;     // 正常，允许 accel 慢慢修正
    }
    //4.写Kalman增益Kt
    //分母为H * Pt * HT + R
    float S = kf->Pt[0][0] + kf->R_measure;
    if (S <= 1.0e-12f) {
        return kf->angle_rad;
    }//防止分母过小、增益过大引起数值不稳定

    //分子为Pt * HT, 这里H是[1 0], 所以分子就是Pt的第一列
    //直接写增益
    float Kt0 = kf->Pt[0][0] / S;
    float Kt1 = kf->Pt[1][0] / S;

    //保存调试量
    kf->K0 = Kt0;
    kf->K1 = Kt1;

    //5.更新最优估计值
    kf->angle_rad  += Kt0 * yt;
    kf->bias_radps += Kt1 * yt;
    //注意这里 bias 也被残差修正。虽然测量值只测角度，但因为 P[1][0] 反映了 bias 和 angle 误差的相关性，所以 K1 可以修正 bias。

    kf->angle_degree = kf->angle_rad * 180.0f / 3.14159f; // 仅供调试用
    //6.更新后验协方差矩阵
    //这里一定要先保存旧的 P00 和 P01
    float P00_temp = kf->Pt[0][0];
    float P01_temp = kf->Pt[0][1];

    //这里最容易犯的错是：不保存临时变量，导致前面改了 P[0][0]，后面又用到已经被改过的值。
    kf->Pt[0][0] -= Kt0 * P00_temp;
    kf->Pt[0][1] -= Kt0 * P01_temp;
    kf->Pt[1][0] -= Kt1 * P00_temp;
    kf->Pt[1][1] -= Kt1 * P01_temp;

    return kf->angle_rad;
}
