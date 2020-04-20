/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <units/units.h>

#include "frc/system/LinearSystem.h"

namespace frc {

/**
 * Constructs the state-space model for a 1 DOF velocity-only system from system
 * identification data.
 *
 * States: [[velocity]]
 * Inputs: [[voltage]]
 * Outputs: [[velocity]]
 *
 * The parameters provided by the user are from this feedforward model:
 *
 * u = K_v v + K_a a
 *
 * @param kV The velocity gain, in volt seconds per distance.
 * @param kA The acceleration gain, in volt seconds^2 per distance.
 * @param maxVoltage The max voltage that can be applied. Inputs with
 *                   greater magnitude than this will be clamped to it.
 */
template <class Distance>
LinearSystem<1, 1, 1> IdentifyVelocitySystem(
    units::unit_t<units::compound_unit<
        units::volts, units::inverse<units::compound_unit<
                          Distance, units::inverse<units::seconds>>>>>
        kV,
    units::unit_t<units::compound_unit<
        units::volts,
        units::inverse<units::compound_unit<
            units::compound_unit<Distance, units::inverse<units::seconds>>,
            units::inverse<units::seconds>>>>>
        kA,
    units::volt_t maxVoltage);

/**
 * Constructs the state-space model for a 1 DOF position system from system
 * identification data.
 *
 * States: [[position], [velocity]]
 * Inputs: [[voltage]]
 * Outputs: [[position]]
 *
 * The parameters provided by the user are from this feedforward model:
 *
 * u = K_v v + K_a a
 *
 * @param kV The velocity gain, in volt seconds per distance.
 * @param kA The acceleration gain, in volt seconds^2 per distance.
 * @param maxVoltage The max voltage that can be applied. Inputs with
 *                   greater magnitude than this will be clamped to it.
 */
template <class Distance>
LinearSystem<2, 1, 1> IdentifyPositionSystem(
    units::unit_t<units::compound_unit<
        units::volts, units::inverse<units::compound_unit<
                          Distance, units::inverse<units::seconds>>>>>
        kV,
    units::unit_t<units::compound_unit<
        units::volts,
        units::inverse<units::compound_unit<
            units::compound_unit<Distance, units::inverse<units::seconds>>,
            units::inverse<units::seconds>>>>>
        kA,
    units::volt_t maxVoltage);

/**
 * Constructs the state-space model for a 2 DOF drivetrain velocity system from
 * system identification data.
 *
 * States: [[left velocity], [right velocity]]
 * Inputs: [[left voltage], [right voltage]]
 * Outputs: [[left velocity], [right velocity]]
 *
 * @param kVlinear The linear velocity gain, in volt seconds per distance.
 * @param kAlinear The linear acceleration gain, in volt seconds^2 per distance.
 * @param kVangular The angular velocity gain, in volt seconds per angle.
 * @param kAangular The angular acceleration gain, in volt seconds^2 per angle.
 * @param maxVoltage the maximum voltage that can be applied.
 */
template <class Distance>
LinearSystem<2, 2, 2> IdentifyDrivetrainSystem(
    units::unit_t<units::compound_unit<
        units::volts, units::inverse<units::compound_unit<
                          Distance, units::inverse<units::seconds>>>>>
        kVlinear,
    units::unit_t<units::compound_unit<
        units::volts,
        units::inverse<units::compound_unit<
            units::compound_unit<Distance, units::inverse<units::seconds>>,
            units::inverse<units::seconds>>>>>
        kAlinear,
    units::unit_t<units::compound_unit<
        units::volts, units::inverse<units::compound_unit<
                          Distance, units::inverse<units::seconds>>>>>
        kVangular,
    units::unit_t<units::compound_unit<
        units::volts,
        units::inverse<units::compound_unit<
            units::compound_unit<Distance, units::inverse<units::seconds>>,
            units::inverse<units::seconds>>>>>
        kAangular,
    units::volt_t maxVoltage);

}  // namespace frc
