// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frc5010.common.motors;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

class SystemIdentificationTest {

  @Test
  void testComputeFeedforwardCoefficientsWithAcceleration() {
    double kS = 0.2;
    double kV = 1.5;
    double kA = 0.3;
    double dt = 0.02;

    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();
    List<Double> timeSamples = new ArrayList<>();

    double time = 0.0;
    double velocity = 0.0;
    for (int i = 0; i < 120; i++) {
      double accel = 0.6 + 0.25 * Math.sin(i * 0.15);
      velocity += accel * dt;
      double voltage = kS + kV * velocity + kA * accel;
      velocitySamples.add(velocity);
      voltageSamples.add(voltage);
      timeSamples.add(time);
      time += dt;
    }

    double[] coefficients =
        SystemIdentification.computeFeedforwardCoefficients(
            velocitySamples, voltageSamples, timeSamples);

    assertNotNull(coefficients);
    assertEquals(kS, coefficients[0], 1e-2);
    assertEquals(kV, coefficients[1], 1e-2);
    assertEquals(kA, coefficients[2], 1e-2);
  }

  @Test
  void testComputeFeedforwardCoefficientsWithoutAccelerationData() {
    double kS = 0.1;
    double kV = 2.0;
    double dt = 0.02;

    List<Double> velocitySamples = new ArrayList<>();
    List<Double> voltageSamples = new ArrayList<>();
    List<Double> timeSamples = new ArrayList<>();

    double velocity = 0.0;
    double time = 0.0;
    for (int i = 0; i < 30; i++) {
      double voltage = kS + kV * velocity;
      velocitySamples.add(velocity);
      voltageSamples.add(voltage);
      timeSamples.add(time);
      velocity += 0.25;
      time += dt;
    }

    double[] coefficients =
        SystemIdentification.computeFeedforwardCoefficients(velocitySamples, voltageSamples, null);

    assertNotNull(coefficients);
    assertEquals(kS, coefficients[0], 1e-6);
    assertEquals(kV, coefficients[1], 1e-6);
    assertEquals(0.0, coefficients[2], 1e-9);
  }
}
