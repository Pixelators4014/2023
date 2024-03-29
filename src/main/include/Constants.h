// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// all units
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/area.h>
#include <units/capacitance.h>
#include <units/charge.h>
#include <units/concentration.h>
#include <units/conductance.h>
#include <units/current.h>
#include <units/curvature.h>
#include <units/data.h>
#include <units/data_transfer_rate.h>
#include <units/density.h>
#include <units/dimensionless.h>
#include <units/energy.h>
#include <units/force.h>
#include <units/frequency.h>
#include <units/illuminance.h>
#include <units/impedance.h>
#include <units/inductance.h>
#include <units/length.h>
#include <units/luminous_flux.h>
#include <units/luminous_intensity.h>
#include <units/magnetic_field_strength.h>
#include <units/magnetic_flux.h>
#include <units/mass.h>
#include <units/moment_of_inertia.h>
#include <units/power.h>
#include <units/pressure.h>
#include <units/radiation.h>
#include <units/solid_angle.h>
#include <units/substance.h>
#include <units/temperature.h>
#include <units/time.h>
#include <units/torque.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <units/volume.h>
#include <units/constants.h>
#include <frc/kinematics/DifferentialDriveKinematics.h>

#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>

#pragma once

namespace DriveConstants {
constexpr auto kDrivetrainGearing = 8.0;
constexpr auto kMOI = 7.5_kg_sq_m;
constexpr auto kMass = 30_kg;
constexpr auto kWheelDiameter = 6_in;
constexpr auto kTrackWidth = 0.7112_m;
constexpr auto kLeftMasterID = 11;
constexpr auto kLeftFollower1ID = 12;
constexpr auto kLeftFollower2ID = 13;
constexpr auto kRightMasterID = 14;
constexpr auto kRightFollower1ID = 15;
constexpr auto kRightFollower2ID = 16;
constexpr auto kLeftMotorInverted = true;
constexpr auto kRightMotorInverted = false;

constexpr auto ks = 0.22_V;
constexpr auto kv = 1.98 * 1_V / 1_mps;
constexpr auto ka = 0.2 * 1_V / 1_mps_sq;

constexpr double kPDriveVel = 8.5;

extern const frc::DifferentialDriveKinematics kDriveKinematics;

constexpr auto kCountsPerRevolution = 2048;
constexpr auto kWheelEncoderMetersPerUnit = units::meter_t{kWheelDiameter * units::constants::pi / (kCountsPerRevolution * kDrivetrainGearing)};
constexpr auto kNeutralMode = NeutralMode::Coast;
}

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 3_mps_sq;
constexpr auto kRamseteB = 2 * 1_rad * 1_rad / (1_m * 1_m);
constexpr auto kRamseteZeta = 0.7 / 1_rad;

// values docs: .1,.1,.2 very strong reistance force but overcomponsated for a slightly bent platform ( looks like it will oscolate on the real platform )

constexpr auto kFP = .003;
constexpr auto kFI = .001;
constexpr auto kFD = .001;

constexpr auto kRP = .003;
constexpr auto kRI = .001;
constexpr auto kRD = .001;

constexpr auto kMotorStrength = .62;
}

namespace ArmConstants {
constexpr auto kJ1ID = 0;
constexpr auto kJ2ID = 1;
constexpr auto kJ3ID = 2;
constexpr auto kJ4ID = 3;

constexpr auto kL1 = 0.8636;
constexpr auto kL2 = 0.8636;
constexpr auto kL3 = 0.8636;

constexpr auto Qelems = 8.0;
constexpr auto Relems = 12.0;

constexpr auto kFalconNeutralMode = NeutralMode::Brake;
constexpr auto kRevNeutralMode = rev::CANSparkMax::IdleMode::kBrake;

constexpr auto kStallTorque = 3.28_Nm;
constexpr auto kFreeSpeed = 5820_rpm;
}

namespace GrabberConstants {
constexpr auto kLeftID = 30;
constexpr auto kRightID = 31;

constexpr auto kLeftInverted = false;
constexpr auto kRightInverted = false;

constexpr auto kLeftPiston1ID = 8;
constexpr auto kLeftPiston2ID = 9;
constexpr auto kRightPiston1ID = 10;
constexpr auto kRightPiston2ID = 11;

constexpr auto kP = 0.6;
constexpr auto kI = 0.0;
constexpr auto kD = 0.0;

constexpr auto kNeutralMode = rev::CANSparkMax::IdleMode::kBrake;
}

namespace Pnumatics {
constexpr auto minPressure = 0_psi;
constexpr auto maxPressure = 120_psi;

constexpr auto kPHID = 32;
}

namespace OIConstants {
constexpr auto driverControllerPort = 0;
constexpr auto operatorControllerPort = 1;
constexpr auto driverControllerHalfRotationSpeedButton = 1;
constexpr auto driverControllerBrakeModeButton = 7;
constexpr auto driverControllerCoastModeButton = 8;
}