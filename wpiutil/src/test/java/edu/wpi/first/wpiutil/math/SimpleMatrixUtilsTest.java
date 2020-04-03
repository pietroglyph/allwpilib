/*----------------------------------------------------------------------------*/
/* Copyright (c) 2020 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpiutil.math;

import org.ejml.simple.SimpleMatrix;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;

public class SimpleMatrixUtilsTest {
  @Test
  public void testLLTDecomposeAllZeros() {
    var src = SimpleMatrix.diag(0.0, 0.0, 0.0);

    assertDoesNotThrow(() -> SimpleMatrixUtils.lltDecompose(src));
  }

}
