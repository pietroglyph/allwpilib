/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.estimator;

import org.ejml.EjmlUnitTests;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.VecBuilder;

public class MerweScaledSigmaPointsTest {
  @Test
  public void testZeroMeanPoints() {
    var merweScaledSigmaPoints = new MerweScaledSigmaPoints<>(Nat.N2());
    var points = merweScaledSigmaPoints.sigmaPoints(VecBuilder.fill(0, 0),
          new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1, 0, 0, 1));

    assertMatrixEquals(points, new MatBuilder<>(Nat.N5(), Nat.N2()).fill(0, 0, 0.00173205, 0, 0,
          0.00173205, -0.00173205, 0, 0, -0.00173205));
  }

  @Test
  public void testNonzeroMeanPoints() {
    var merweScaledSigmaPoints = new MerweScaledSigmaPoints<>(Nat.N2());
    var points = merweScaledSigmaPoints.sigmaPoints(VecBuilder.fill(1, 2),
          new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1, 0, 0, 10));

    assertMatrixEquals(points, new MatBuilder<>(Nat.N5(), Nat.N2()).fill(1, 2, 1.00173205, 2, 1,
          2.00547723, 0.99826795, 2, 1, 1.99452277));
  }

  @SuppressWarnings("ParameterName")
  void assertMatrixEquals(Matrix a, Matrix b) {
    EjmlUnitTests.assertEquals(a.getStorage().getDDRM(), b.getStorage().getDDRM());
  }

}