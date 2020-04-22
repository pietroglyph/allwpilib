/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include <gtest/gtest.h>

#include <drake/common/test_utilities/eigen_matrix_compare.h>

#include "frc/StateSpaceUtil.h"
#include "frc/estimator/MerweScaledSigmaPoints.h"

namespace drake {
namespace math {
namespace {
TEST(MerweScaledSigmaPointsTest, TestZeroMean) {
  frc::MerweScaledSigmaPoints<2> sigmaPoints{};
  auto points =
      sigmaPoints.SigmaPoints(frc::MakeMatrix<2, 1>(0.0, 0.0),
                              frc::MakeMatrix<2, 2>(1.0, 0.0, 0.0, 1.0));

  EXPECT_TRUE(CompareMatrices(
      points,
      frc::MakeMatrix<5, 2>(0.0, 0.0, 0.00173205, 0.0, 0.0, 0.00173205,
                            -0.00173205, 0.0, 0.0, -0.00173205),
      1E-4, MatrixCompareType::absolute));
}

TEST(MerweScaledSigmaPointsTest, TestNonzeroMean) {
  frc::MerweScaledSigmaPoints<2> sigmaPoints{};
  auto points =
      sigmaPoints.SigmaPoints(frc::MakeMatrix<2, 1>(1.0, 2.0),
                              frc::MakeMatrix<2, 2>(1.0, 0.0, 0.0, 10.0));

  EXPECT_TRUE(CompareMatrices(
      points,
      frc::MakeMatrix<5, 2>(1.0, 2.0, 1.00173205, 2.0, 1.0, 2.00547723,
                            0.99826795, 2.0, 1.0, 1.99452277),
      1E-4, MatrixCompareType::absolute));
}
}  // namespace
}  // namespace math
}  // namespace drake
