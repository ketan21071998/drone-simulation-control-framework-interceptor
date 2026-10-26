#pragma once

#define _USE_MATH_DEFINES
#include <cmath>
constexpr double PI = 3.14159265358979323846;

#include <array>
#include <mutex>
#include <algorithm>
#include <chrono>
#include <vector>

// =========================================================
// Vec3 – minimal 3-D vector
// =========================================================
struct Vec3 {
    double x{0}, y{0}, z{0};

    Vec3() = default;
    Vec3(double x, double y, double z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(double s)      const { return {x*s,   y*s,   z*s};   }
    Vec3 operator/(double s)      const { return {x/s,   y/s,   z/s};   }
    Vec3& operator+=(const Vec3& o){ x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(double s)     { x*=s;   y*=s;   z*=s;   return *this; }

    double dot(const Vec3& o) const { return x*o.x + y*o.y + z*o.z; }
    Vec3 cross(const Vec3& o) const {
        return { y*o.z - z*o.y,
                 z*o.x - x*o.z,
                 x*o.y - y*o.x };
    }
    double length() const { return std::sqrt(x*x + y*y + z*z); }
};
inline Vec3 operator*(double s, const Vec3& v){ return v*s; }

// =========================================================
// Mat3 – 3×3 rotation matrix (row-major)
// =========================================================
struct Mat3 {
    double m[3][3]{};

    static Mat3 identity() {
        Mat3 r; r.m[0][0]=r.m[1][1]=r.m[2][2]=1; return r;
    }

    Vec3 operator*(const Vec3& v) const {
        return { m[0][0]*v.x + m[0][1]*v.y + m[0][2]*v.z,
                 m[1][0]*v.x + m[1][1]*v.y + m[1][2]*v.z,
                 m[2][0]*v.x + m[2][1]*v.y + m[2][2]*v.z };
    }

    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for(int i=0;i<3;i++)
          for(int j=0;j<3;j++)
            for(int k=0;k<3;k++)
              r.m[i][j] += m[i][k]*o.m[k][j];
        return r;
    }

    Mat3 T() const {   // transpose
        Mat3 r;
        for(int i=0;i<3;i++) for(int j=0;j<3;j++) r.m[i][j]=m[j][i];
        return r;
    }
};

// =========================================================
// Physical constants  (mirrors Python exactly)
// =========================================================
constexpr double MASS               = 1.5;
constexpr double GRAVITY            = 9.81;
constexpr double MAX_THRUST         = 20.0;
constexpr double MAX_RPM            = 20000.0;
constexpr double RPM_RESPONSE       = 0.08;
constexpr double ARM_LEN            = 0.25;
constexpr double DRAG_COEFF         = 0.01;
constexpr double YAW_COEFF          = 0.015;
constexpr double ROT_DAMPING_120HZ  = 0.9965;
constexpr double SIM_HZ             = 120.0;
constexpr double DT                 = 1.0 / SIM_HZ;

inline Vec3 INERTIA() { return {0.02, 0.04, 0.02}; }

// =========================================================
// Clamp PWM [0,1]
// =========================================================
inline double clamp01(double v){ return std::clamp(v, 0.0, 1.0); }


// =========================================================
// Euler → Rotation matrix  (EUS, same order as Python)
// pitch→Rx  yaw→Ry  roll→Rz,  result = Rz*Ry*Rx
// =========================================================
inline Mat3 euler_to_R(double pitch_deg, double yaw_deg, double roll_deg)
{
    auto p = pitch_deg * PI/180.0;
    auto y = yaw_deg   * PI/180.0;
    auto r = roll_deg  * PI/180.0;

    Mat3 Rx; Rx.m[0][0]=1;
    Rx.m[1][1]= std::cos(p); Rx.m[1][2]=-std::sin(p);
    Rx.m[2][1]= std::sin(p); Rx.m[2][2]= std::cos(p);

    Mat3 Ry; Ry.m[1][1]=1;
    Ry.m[0][0]= std::cos(y); Ry.m[0][2]=-std::sin(y);
    Ry.m[2][0]= std::sin(y); Ry.m[2][2]= std::cos(y);

    Mat3 Rz; Rz.m[2][2]=1;
    Rz.m[0][0]= std::cos(r); Rz.m[0][1]=-std::sin(r);
    Rz.m[1][0]= std::sin(r); Rz.m[1][1]= std::cos(r);

    return Rz * Ry * Rx;
}

// =========================================================
// Wrap angle to [-180, 180]
// =========================================================
inline double wrap180(double d){
    d = std::fmod(d + 180.0, 360.0);
    if(d < 0) d += 360.0;
    return d - 180.0;
}

// =========================================================
// Shared simulation state
// =========================================================
struct SimState {
    mutable std::mutex mtx;

    Vec3   position      {0, 2, 0};
    Vec3   velocity      {0, 0, 0};
    Vec3   accel_body    {0, 0, 0};
    Vec3   rotation      {0, 0, 0};   // pitch, yaw, roll  (degrees)
    Vec3   angular_vel   {0, 0, 0};   // body frame rad/s

    int mission_index = 0;
    int current_level = 0;

    std::vector<Vec3> mission_targets =
    {
        {10,10,10},     // Level 1
        {-10,5,10},    // Level 2
        {-10,8,-10},   // Level 3
        {10,10,-10},    // Level 4

        // Moving target levels
        {0,10,0},
        {0,10,0},
        {0,10,0}
    };

    Vec3 target {10,10,10};
    bool target_reached = false;

    std::array<double,4> pwm {0,0,0,0};
    std::array<double,4> rpm {0,0,0,0};

    //std::array<double,4> pwm {0,0,0,0};  // FR FL BR BL

    double sim_time{0.0};   // seconds since reset

    void reset()
    {
        std::lock_guard<std::mutex> lk(mtx);

        position    = {0,2,0};
        velocity    = {0,0,0};
        accel_body  = {0,0,0};
        rotation    = {0,0,0};
        angular_vel = {0,0,0};

        pwm         = {0,0,0,0};
        rpm         = {0,0,0,0};

        mission_index = 0;
        target = mission_targets[0];
        target_reached = false;


        sim_time = 0.0;
    }

    // Write PWM from network thread
    void set_pwm(const std::array<double,4>& p) {
        std::lock_guard<std::mutex> lk(mtx);
        for(int i=0;i<4;i++) pwm[i]=clamp01(p[i]);
    }
};

// =========================================================
// Physics step  – call at 120 Hz
// =========================================================
inline void physics_step(SimState& s)
{
    std::lock_guard<std::mutex> lk(s.mtx);

    for(int i=0;i<4;i++)
    {
        double target_rpm =
            s.pwm[i] * MAX_RPM;

        s.rpm[i] +=
            (target_rpm - s.rpm[i]) *
            RPM_RESPONSE;
    }

    double FR=s.pwm[0];
    double FL=s.pwm[1];
    double BR=s.pwm[2];
    double BL=s.pwm[3];

    double thrusts[4] =
    {
        s.pwm[0] * MAX_THRUST,   // FR
        s.pwm[1] * MAX_THRUST,   // FL
        s.pwm[2] * MAX_THRUST,   // BR
        s.pwm[3] * MAX_THRUST    // BL
    };
    double total_thrust = thrusts[0]+thrusts[1]+thrusts[2]+thrusts[3];

    Mat3 R = euler_to_R(s.rotation.x, s.rotation.y, s.rotation.z);

    // Force in world frame
    Vec3 force_body {0, total_thrust, 0};
    Vec3 force_world = R * force_body;
    Vec3 gravity     {0, -MASS*GRAVITY, 0};
    Vec3 drag        = s.velocity * (-DRAG_COEFF);

    Vec3 accel_world = (force_world + gravity + drag) * (1.0/MASS);

    s.velocity += accel_world * DT;
    s.position += s.velocity * DT;
    s.accel_body = R.T() * accel_world;

    // Motor positions in body frame
    Vec3 motor_pos[4] = {
        { ARM_LEN, 0,  -ARM_LEN},  // FR
        {-ARM_LEN, 0,  -ARM_LEN},  // FL
        { ARM_LEN, 0,   ARM_LEN},  // BR
        {-ARM_LEN, 0,   ARM_LEN}   // BL
    };

    Vec3 torque {0,0,0};
    for(int i=0;i<4;i++){
        Vec3 f {0, thrusts[i], 0};
        torque += motor_pos[i].cross(f);
    }
    // Yaw reaction torque
    double yaw_torque =
    YAW_COEFF *
    ((thrusts[0] + thrusts[3]) -
     (thrusts[1] + thrusts[2]));
    torque.y += yaw_torque;

    Vec3 I = INERTIA();
    Vec3 ang_accel { torque.x/I.x, torque.y/I.y, torque.z/I.z };
    s.angular_vel += ang_accel * DT;

    double damp = std::pow(ROT_DAMPING_120HZ, DT/(1.0/120.0));
    s.angular_vel *= damp;

    double deg = 180.0/PI;
    s.rotation.x = wrap180(s.rotation.x + s.angular_vel.x*DT*deg);
    s.rotation.y = wrap180(s.rotation.y + s.angular_vel.y*DT*deg);
    s.rotation.z = wrap180(s.rotation.z + s.angular_vel.z*DT*deg);

    // Ground collision
    if(s.position.y < 0.0){
        s.position.y = 0.0;
        if(s.velocity.y < 0) s.velocity.y = 0.0;
        s.angular_vel *= 0.5;
    }

    // =====================================================
    // Mission manager
    // =====================================================

    Vec3 diff =
    {
        s.position.x - s.target.x,
        s.position.y - s.target.y,
        s.position.z - s.target.z
    };

    bool reached =
        std::abs(diff.x) < 0.4 &&
        std::abs(diff.y) < 0.4 &&
        std::abs(diff.z) < 0.4;

    if(reached)
    {
        s.mission_index++;

        if(s.mission_index >= (int)s.mission_targets.size())
            s.mission_index = (int)s.mission_targets.size() - 1;

        s.target = s.mission_targets[s.mission_index];
    }

    // =====================================================
    // Moving target missions
    // =====================================================

    if(s.mission_index == 4)
    {
        // Move along X axis
        s.target.z = 1.5 * s.sim_time;;
    }

    if(s.mission_index == 5)
    {
        // Move along X axis
        s.target.x = 20.0 * std::sin(0.3 * s.sim_time);
    }

    if(s.mission_index == 6)
    {
        // Circle
        s.target.x = 20.0 * std::cos(0.3 * s.sim_time);
        s.target.z = 20.0 * std::sin(0.3 * s.sim_time);
    }

    s.sim_time += DT;
}
