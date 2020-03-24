package edu.wpi.first.wpilibj.math;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Matrix;
import edu.wpi.first.wpiutil.math.Nat;
import edu.wpi.first.wpiutil.math.numbers.N1;
import edu.wpi.first.wpiutil.math.numbers.N2;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;


public class StateSpaceUtilsTest {

  @Test
  public void testCostArray() {
    var mat = StateSpaceUtils.makeCostMatrix(
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(1.0, 2.0, 3.0));

    assertEquals(1.0, mat.get(0, 0), 1e-3);
    assertEquals(0.0, mat.get(0, 1), 1e-3);
    assertEquals(0.0, mat.get(0, 2), 1e-3);
    assertEquals(0.0, mat.get(1, 0), 1e-3);
    assertEquals(1.0 / 4.0, mat.get(1, 1), 1e-3);
    assertEquals(0.0, mat.get(1, 2), 1e-3);
    assertEquals(0.0, mat.get(0, 2), 1e-3);
    assertEquals(0.0, mat.get(1, 2), 1e-3);
    assertEquals(1.0 / 9.0, mat.get(2, 2), 1e-3);
  }

  @Test
  public void testCovArray() {
    var mat = StateSpaceUtils.makeCovMatrix(Nat.N3(),
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(1.0, 2.0, 3.0));

    assertEquals(1.0, mat.get(0, 0), 1e-3);
    assertEquals(0.0, mat.get(0, 1), 1e-3);
    assertEquals(0.0, mat.get(0, 2), 1e-3);
    assertEquals(0.0, mat.get(1, 0), 1e-3);
    assertEquals(4.0, mat.get(1, 1), 1e-3);
    assertEquals(0.0, mat.get(1, 2), 1e-3);
    assertEquals(0.0, mat.get(0, 2), 1e-3);
    assertEquals(0.0, mat.get(1, 2), 1e-3);
    assertEquals(9.0, mat.get(2, 2), 1e-3);
  }

  @Test
  @SuppressWarnings("LocalVariableName")
  public void testIsStabilizable() {
    Matrix<N2, N2> A;
    Matrix<N2, N1> B = new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0, 1);

    // First eigenvalue is uncontrollable and unstable.
    // Second eigenvalue is controllable and stable.
    A = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1.2, 0, 0, 0.5);
    assertFalse(StateSpaceUtils.isStabilizable(A, B));

    // First eigenvalue is uncontrollable and marginally stable.
    // Second eigenvalue is controllable and stable.
    A = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(1, 0, 0, 0.5);
    assertFalse(StateSpaceUtils.isStabilizable(A, B));

    // First eigenvalue is uncontrollable and stable.
    // Second eigenvalue is controllable and stable.
    A = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(0.2, 0, 0, 0.5);
    assertTrue(StateSpaceUtils.isStabilizable(A, B));

    // First eigenvalue is uncontrollable and stable.
    // Second eigenvalue is controllable and unstable.
    A = new MatBuilder<>(Nat.N2(), Nat.N2()).fill(0.2, 0, 0, 1.2);
    assertTrue(StateSpaceUtils.isStabilizable(A, B));
  }

}
