#include <Eigen/Dense>

using namespace Eigen;
typedef Matrix<double, 9, 1> Vector9d;

namespace Kinematics {

Matrix4d ForwardKinematics(const double *q, double tool_length = 0.173) {
    // Vector7d q

    Matrix4d Ab0, A01, A12, A23, A34, A45, A56, A67, A7f, Afr;
    Matrix4d Ab1, Ab2, Ab3, Ab4, Ab5, Ab6, Ab7, Abe;
    Matrix3d R1, R2, R3, R4, R5, R6, R7;
    Vector3d z0, z1, z2, z3, z4, z5, z6, z7, zer;
    Vector3d p0, p1, p2, p3, p4, p5, p6, p7, pe;
    zer.setZero();  // zero vector
    double C1, S1, C2, S2, C3, S3, C4, S4, C5, S5, C6, S6, C7, S7;
    double d1{0.333}, d3{0.316}, d5{0.384}, df{0.107};
    double a3{0.0825}, a4{-0.0825}, a6{0.088};
    double l(tool_length);  // tool length after panda_link8

    C1 = cos(q[0]);
    S1 = sin(q[0]);
    C2 = cos(q[1]);
    S2 = sin(q[1]);
    C3 = cos(q[2]);
    S3 = sin(q[2]);
    C4 = cos(q[3]);
    S4 = sin(q[3]);
    C5 = cos(q[4]);
    S5 = sin(q[4]);
    C6 = cos(q[5]);
    S6 = sin(q[5]);
    C7 = cos(q[6]);
    S7 = sin(q[6]);

    Ab0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    A01 << C1, -S1, 0, 0,
        S1, C1, 0, 0,
        0, 0, 1, d1,
        0, 0, 0, 1;

    A12 << C2, -S2, 0, 0,
        0, 0, 1, 0,
        -S2, -C2, 0, 0,
        0, 0, 0, 1;

    A23 << C3, -S3, 0, 0,
        0, 0, -1, -d3,
        S3, C3, 0, 0,
        0, 0, 0, 1;

    A34 << C4, -S4, 0, a3,
        0, 0, -1, 0,
        S4, C4, 0, 0,
        0, 0, 0, 1;

    A45 << C5, -S5, 0, a4,
        0, 0, 1, d5,
        -S5, -C5, 0, 0,
        0, 0, 0, 1;

    A56 << C6, -S6, 0, 0,
        0, 0, -1, 0,
        S6, C6, 0, 0,
        0, 0, 0, 1;

    A67 << C7, -S7, 0, a6,
        0, 0, -1, 0,
        S7, C7, 0, 0,
        0, 0, 0, 1;

    A7f << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, df + l,
        0, 0, 0, 1;  // d7+l

    return Ab0 * A01 * A12 * A23 * A34 * A45 * A56 * A67 * A7f;  //*Afr;
}

//**************************************************************************************
Matrix<double, 6, 7> ComputeJacobian(const double *q, double tool_length = 0.173) {
    Matrix<double, 6, 7> Jacobian;
    Jacobian.setZero();

    Matrix4d XX, Ab0, A01, A12, A23, A34, A45, A56, A67, A7e;
    Matrix4d Ab1, Ab2, Ab3, Ab4, Ab5, Ab6, Ab7, Abe;
    Matrix3d R1, R2, R3, R4, R5, R6, R7;
    Vector3d z0, z1, z2, z3, z4, z5, z6, z7, zer;
    Vector3d p0, p1, p2, p3, p4, p5, p6, p7, pe;
    XX.setZero();
    zer.setZero();  // zero vector
    double C1, S1, C2, S2, C3, S3, C4, S4, C5, S5, C6, S6, C7, S7;
    double d1{0.333}, d3{0.316}, d5{0.384}, df{0.107};
    double a3{0.0825}, a4{-0.0825}, a6{0.088};
    double l(tool_length);  // tool length after panda_link8

    C1 = cos(q[0]);
    S1 = sin(q[0]);
    C2 = cos(q[1]);
    S2 = sin(q[1]);
    C3 = cos(q[2]);
    S3 = sin(q[2]);
    C4 = cos(q[3]);
    S4 = sin(q[3]);
    C5 = cos(q[4]);
    S5 = sin(q[4]);
    C6 = cos(q[5]);
    S6 = sin(q[5]);
    C7 = cos(q[6]);
    S7 = sin(q[6]);

    Ab0 << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, 0,
        0, 0, 0, 1;

    A01 << C1, -S1, 0, 0,
        S1, C1, 0, 0,
        0, 0, 1, d1,
        0, 0, 0, 1;

    A12 << C2, -S2, 0, 0,
        0, 0, 1, 0,
        -S2, -C2, 0, 0,
        0, 0, 0, 1;

    A23 << C3, -S3, 0, 0,
        0, 0, -1, -d3,
        S3, C3, 0, 0,
        0, 0, 0, 1;

    A34 << C4, -S4, 0, a3,
        0, 0, -1, 0,
        S4, C4, 0, 0,
        0, 0, 0, 1;

    A45 << C5, -S5, 0, a4,
        0, 0, 1, d5,
        -S5, -C5, 0, 0,
        0, 0, 0, 1;

    A56 << C6, -S6, 0, 0,
        0, 0, -1, 0,
        S6, C6, 0, 0,
        0, 0, 0, 1;

    A67 << C7, -S7, 0, a6,
        0, 0, -1, 0,
        S7, C7, 0, 0,
        0, 0, 0, 1;

    A7e << 1, 0, 0, 0,
        0, 1, 0, 0,
        0, 0, 1, df + l,
        0, 0, 0, 1; 

    XX = Ab0 * A01 * A12 * A23 * A34 * A45 * A56 * A67 * A7e;

    Ab1 = Ab0 * A01;
    Ab2 = Ab1 * A12;
    Ab3 = Ab2 * A23;
    Ab4 = Ab3 * A34;
    Ab5 = Ab4 * A45;
    Ab6 = Ab5 * A56;
    Ab7 = Ab6 * A67;
    Abe = Ab7 * A7e;

    R1 = Ab1.block<3, 3>(0, 0);
    R2 = Ab2.block<3, 3>(0, 0);
    R3 = Ab3.block<3, 3>(0, 0);
    R4 = Ab4.block<3, 3>(0, 0);
    R5 = Ab5.block<3, 3>(0, 0);
    R6 = Ab6.block<3, 3>(0, 0);
    R7 = Ab7.block<3, 3>(0, 0);

    z0 = Ab0.block<3, 1>(0, 2);
    z1 = Ab1.block<3, 1>(0, 2);
    z2 = Ab2.block<3, 1>(0, 2);
    z3 = Ab3.block<3, 1>(0, 2);
    z4 = Ab4.block<3, 1>(0, 2);
    z5 = Ab5.block<3, 1>(0, 2);
    z6 = Ab6.block<3, 1>(0, 2);
    z7 = Ab7.block<3, 1>(0, 2);

    p0 = Ab0.block<3, 1>(0, 3);
    p1 = Ab1.block<3, 1>(0, 3);
    p2 = Ab2.block<3, 1>(0, 3);
    p3 = Ab3.block<3, 1>(0, 3);
    p4 = Ab4.block<3, 1>(0, 3);
    p5 = Ab5.block<3, 1>(0, 3);
    p6 = Ab6.block<3, 1>(0, 3);
    p7 = Ab7.block<3, 1>(0, 3);
    pe = XX.block<3, 1>(0, 3);

    Jacobian << z1.cross(pe - p1), z2.cross(pe - p2), z3.cross(pe - p3), z4.cross(pe - p4), z5.cross(pe - p5), z6.cross(pe - p6), z7.cross(pe - p7),
        z1, z2, z3, z4, z5, z6, z7;

    return Jacobian;
}

void spiralTrajectory(double t, Vector3d &p0, Vector9d &des) {
    double xi[3];
    double L, v_max, T, a_max, Tm, S, Sd, Sdd;

    xi[0] = p0[0];
    xi[1] = p0[1];
    xi[2] = p0[2];

    // timing law

    T = 15;

    double pitch = 0.0;
    double R = 0.10;
    L = 4 * M_PI;

    v_max = 1.3 * L / T;

    a_max = v_max * v_max / (T * v_max - L);
    Tm = v_max / a_max;

    if (Tm < T / 5 || Tm > T / 2.1) {
        std::cout << "HS: ERROR in trajectory planning timing law" << std::endl;
        exit(1);
    }

    if (t >= 0 && t <= Tm) {
        S = a_max * t * t / 2;
        Sd = a_max * t;
        Sdd = a_max;
    } else if (t >= Tm && t <= (T - Tm)) {
        S = v_max * t - v_max * v_max / (2 * a_max);
        Sd = v_max;
        Sdd = 0;
    } else if (t >= (T - Tm) && t <= T) {
        S = -a_max * (t - T) * (t - T) / 2 + v_max * T - v_max * v_max / a_max;
        Sd = -a_max * (t - T);
        Sdd = -a_max;
    } else {
        S = L;
        Sd = 0;
        Sdd = 0;
    }
    // Geometric path
    //  spiral with z axis
    // p_des
    des[0] = (xi[0] - R) + R * cos(S);
    des[1] = xi[1] + R * sin(S);
    des[2] = xi[2] + S * pitch / (2 * M_PI);
    // pd_des
    des[3] = -R * Sd * sin(S);
    des[4] = R * Sd * cos(S);
    des[5] = pitch * Sd / (2 * M_PI);
    // pdd_des
    des[6] = -R * Sdd * sin(S) - R * Sd * Sd * cos(S);
    des[7] = R * Sdd * cos(S) - R * Sd * Sd * sin(S);
    des[8] = pitch * Sdd / (2 * M_PI);

    //// spiral with y axis
    // p_des
    // des[0] = (xi[0] - R) + R*cos(S);
    // des[1] = xi[1] + S*pitch / (2 * M_PI);
    // des[2] = xi[2] + R*sin(S);
    ////pd_des
    // des[3] = -R*Sd*sin(S);
    // des[4] = pitch*Sd / (2 * M_PI);
    // des[5] = R*Sd*cos(S);
    ////pdd_des
    // des[6] = 0;
    // des[7] = 0;
    // des[8] = 0;
    //
}

void trajectory(double t, Vector3d &p0, Vector9d &des) {
    constexpr double kRadius = 0.1;
    double T = 10;
    double a = 1.0;

    double angle = M_PI * 2 * (1 - std::cos(M_PI / T * t));
    if (t >= T) {
        angle = 0.0;
        a = 0.0;
    }
    // double delta_x = kRadius * std::sin(angle);
    // double delta_z = kRadius * (std::cos(angle) - 1);
    double delta_x = -kRadius * (std::cos(angle) - 1);
    double delta_y = kRadius * std::sin(angle);
    double omega = M_PI * 2 * std::sin(M_PI / T * t) * M_PI / T;

    double v_x = kRadius * std::sin(angle) * omega;
    double v_y = kRadius * std::cos(angle) * omega;

    // p_des
    des[0] = p0[0] + delta_x;
    des[1] = p0[1] + delta_y;
    des[2] = p0[2];
    // pd_des
    des[3] = a * v_x;
    des[4] = a * v_y;
    des[5] = 0.0;
    // pdd_des
    des[6] = 0;
    des[7] = 0;
    des[8] = 0;
}

}
