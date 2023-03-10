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

#include <ctre/Phoenix.h>

#pragma once

namespace DriveConstants {
constexpr auto kTrackwidth = 0.69_m;
constexpr auto kDrivetrainGearing = 8.0;
constexpr auto kMOI = 7.5_kg_sq_m;
constexpr auto kMass = 60_kg;
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


// constexpr auto kCountsPerRevolution = 2048;
// constexpr auto kEncoderUnitsPerInch = 1;
constexpr auto kWheelEncoderMetersPerUnit = 1_m;
constexpr auto kNeutralMode = NeutralMode::Coast;
}

namespace ArmConstants {
constexpr auto kJ1ID = 0;
constexpr auto kJ2ID = 1;
constexpr auto kJ3ID = 2;
constexpr auto kJ4ID = 3;

constexpr auto kL1 = 0.1_m;
constexpr auto kL2 = 0.1_m;
constexpr auto kL3 = 0.1_m;
constexpr auto kL4 = 0.1_m;

constexpr auto kNeutralMode = NeutralMode::Coast;
}

namespace OIConstants {
constexpr auto driverControllerPort = 1;
}