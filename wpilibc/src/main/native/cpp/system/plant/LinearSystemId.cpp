/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "frc/system/plant/LinearSystemId.h"

#include "frc/StateSpaceUtil.h"

namespace frc {

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
    units::volt_t maxVoltage) {
  auto A = frc::MakeMatrix<1, 1>(-kV.to<double>() / kA.to<double>());
  auto B = frc::MakeMatrix<1, 1>(1.0 / kA.to<double>());
  auto C = frc::MakeMatrix<1, 1>(1.0);
  auto D = frc::MakeMatrix<1, 1>(0.0);
  auto uMin = frc::MakeMatrix<1, 1>(-maxVoltage.to<double>());
  auto uMax = frc::MakeMatrix<1, 1>(maxVoltage.to<double>());

  return LinearSystem<1, 1, 1>(A, B, C, D, uMin, uMax);
}

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
    units::volt_t maxVoltage) {
  auto A =
      frc::MakeMatrix<2, 2>(0.0, 1.0, 0.0, -kV.to<double>() / kA.to<double>());
  auto B = frc::MakeMatrix<2, 1>(0.0, 1.0 / kA.to<double>());
  auto C = frc::MakeMatrix<1, 2>(1.0, 0.0);
  auto D = frc::MakeMatrix<1, 1>(0.0);
  auto uMin = frc::MakeMatrix<1, 1>(-maxVoltage.to<double>());
  auto uMax = frc::MakeMatrix<1, 1>(maxVoltage.to<double>());

  return LinearSystem<2, 1, 1>(A, B, C, D, uMin, uMax);
}

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
    units::volt_t maxVoltage) {
  double c = 0.5 / (kAlinear.to<double>() * kAangular.to<double>());
  double A1 = c * (-kAlinear.to<double>() * kVangular.to<double>() -
                   kVlinear.to<double>() * kAangular.to<double>());
  double A2 = c * (kAlinear.to<double>() * kVangular.to<double>() -
                   kVlinear.to<double>() * kAangular.to<double>());
  double B1 = c * (kAlinear.to<double>() + kAangular.to<double>());
  double B2 = c * (kAangular.to<double>() - kAlinear.to<double>());

  auto A = frc::MakeMatrix<2, 2>(A1, A2, A2, A1);
  auto B = frc::MakeMatrix<2, 2>(B1, B2, B2, B1);
  auto C = frc::MakeMatrix<2, 2>(1.0, 0.0, 0.0, 1.0);
  auto D = frc::MakeMatrix<2, 2>(0.0, 0.0, 0.0, 0.0);
  auto uMin =
      frc::MakeMatrix<2, 1>(-maxVoltage.to<double>(), -maxVoltage.to<double>());
  auto uMax =
      frc::MakeMatrix<2, 1>(maxVoltage.to<double>(), maxVoltage.to<double>());

  return LinearSystem<2, 2, 2>(A, B, C, D, uMin, uMax);
}

}  // namespace frc
