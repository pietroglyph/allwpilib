package edu.wpi.first.wpilibj.system;

import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpiutil.math.MatBuilder;
import edu.wpi.first.wpiutil.math.Nat;
import org.junit.jupiter.api.Test;

import static org.ejml.EjmlUnitTests.assertEquals;

class LinearSystemTest {
  @Test
  public void testDrivetrainVelcitySystem() {
    var model = LinearSystem.createDrivetrainVelocitySystem(
            DCMotor.getNEO(4), 70, 0.05, 0.4, 6.0, 6, 12
    );
    assertEquals(model.getA().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(-10.14132, 3.06598, 3.06598, -10.14132).getStorage().getDDRM(), 0.001);

    assertEquals(model.getB().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(4.2590, -1.28762, -1.2876, 4.2590).getStorage().getDDRM(), 0.001);

    assertEquals(model.getC().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(1.0, 0.0, 0.0, 1.0).getStorage().getDDRM(), 0.001);

    assertEquals(model.getD().getStorage().getDDRM(), new MatBuilder<>(Nat.N2(),
            Nat.N2()).fill(0.0, 0.0, 0.0, 0.0).getStorage().getDDRM(), 0.001);
  }

  @Test
  public void testIdentifyDrivetrainSystem() {
    // TODO
  }

  @Test
  public void testIdentifyPositionSystem() {
    // TODO
  }

  @Test
  public void testIdentifyVelocitySystem() {
    // TODO
  }
}
