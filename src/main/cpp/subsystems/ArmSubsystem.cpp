// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem()
{
    m_J1.ConfigFactoryDefault();

    m_J1.SetNeutralMode(kFalconNeutralMode);
    m_J2.SetIdleMode(kRevNeutralMode);
    m_J3.SetIdleMode(kRevNeutralMode);
    m_J4.SetIdleMode(kRevNeutralMode);
};

void ArmSubsystem::moveTo(double x, double y, double z)
{
    double u = sqrt(x*x)-kL3;
    double v = z;
    double theta_1 = atan2(x,y);
    double theta_3 = -acos((u*u+v*v-kL1*kL1-kL2*kL2)/(2*kL1*kL2));
    double theta_2 = atan2(v,u)-atan2(kL2*sin(theta_3),kL1+kL2*cos(theta_3));
    double theta_4 = -theta_3-theta_2;
    moveTo(theta_1,theta_2,theta_3,theta_4);
}

void ArmSubsystem::moveTo(double theta_1, double theta_2, double theta_3, double theta_4)
{
    double ArrayA[64];
    double ArrayB[32];
    A(ArrayA, theta_1, theta_2, theta_3, theta_4, 0, 0, 0, 0, 0, 0, 0, 0);
    B(ArrayB, theta_1, theta_2, theta_3, theta_4, 0, 0, 0, 0, 0, 0, 0, 0);

    Eigen::Map<Eigen::Matrix<double, 8, 8, Eigen::RowMajor>> MatrixA(ArrayA);
    Eigen::Map<Eigen::Matrix<double, 8, 4, Eigen::RowMajor>> MatrixB(ArrayB);

    frc::Matrixd<8, 8> Q = frc::MakeCostMatrix(Qelems);
    frc::Matrixd<4, 4> R = frc::MakeCostMatrix(Relems);

    frc::LinearQuadraticRegulator<8,4> LQR{MatrixA,MatrixB,Q,R, 20_ms};
    // LQR.Calculate();
}
// frc::Pose3d ArmSubsystem::GetPosition(double θ1, double θ2, double θ3, double θ4)
// {
//     units::meter_t x=sin(θ1)*sin(θ2)*kL2+(cos(θ3)*sin(θ1)*sin(θ2)+cos(θ2)*sin(θ1)*sin(θ3))*kL3+(cos(θ4)*(cos(θ3)*sin(θ1)*sin(θ2)+cos(θ2)*sin(θ1)*sin(θ3))-(-cos(θ2)*cos(θ3)*sin(θ1)+sin(θ1)*sin(θ2)*sin(θ3)))*kL4;
//     units::meter_t y=-cos(θ1)*sin(θ2)*kL2+(-cos(θ1)*cos(θ3)*sin(θ2)-cos(θ1)*cos(θ2)*sin(θ3))*kL3+(cos(θ4)*(-cos(θ1)*cos(θ3)*sin(θ2)-cos(θ1)*cos(θ2)*sin(θ3))-(cos(θ1)*cos(θ2)*cos(θ3)-cos(θ1)*sin(θ2)*sin(θ3))*sin(θ4))*kL4;
//     units::meter_t z=kL1+cos(θ2)*kL2+(cos(θ2)*cos(θ3)-sin(θ2)*sin(θ3))*kL2+(cos(θ4)*cos(θ2)*cos(θ3)-sin(θ2)*sin(θ3)-(cos(θ3)*sin(θ2)+cos(θ2)*sin(θ3))*sin(θ4))*kL4;
//     Translation3d T{x, y, z};
// }

// This method will be called once per scheduler run
void ArmSubsystem::Periodic() {}
