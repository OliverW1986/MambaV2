package org.teamtitanium.frc2025.subsystems.swerve;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.teamtitanium.frc2025.utils.PhoenixUtil;

public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyroSimulation;

  public GyroIOSim(GyroSimulation gyroSimulation) {
    this.gyroSimulation = gyroSimulation;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;

    inputs.yawPosition = gyroSimulation.getGyroReading();
    inputs.yawVelocityRadPerSec = gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond);
    // inputs.yawVelocityRadPerSec =
    // Units.degreesToRadians(gyroSimulation.getMeasuredAngularVelocity().in(RadiansPerSecond));

    inputs.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimestamps();
    inputs.odometryYawPositions = gyroSimulation.getCachedGyroReadings();
  }
}
