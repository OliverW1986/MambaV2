package org.teamtitanium.frc2025.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

public class SwerveModuleIOTalonFXReal extends SwerveModuleIOTalonFX {
  private final Queue<Double> timestampQueue;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  @SuppressWarnings({"rawtypes", "unchecked"})
  public SwerveModuleIOTalonFXReal(SwerveModuleConstants constants) {
    super(constants);

    this.timestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    this.drivePositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.drivePosition);
    this.turnPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(super.turnAbsolutePosition);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    super.updateInputs(inputs);

    inputs.odometryTimestamps =
        timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

    timestampQueue.clear();
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }
}
