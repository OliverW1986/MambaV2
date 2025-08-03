package org.teamtitanium.frc2025.subsystems.swerve;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.littletonrobotics.junction.Logger;

@SuppressWarnings("rawtypes")
public class SwerveModule {
  private final SwerveModuleIO io;
  private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

  private final int index;
  private final SwerveModuleConstants constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnCanCoderDisconnectedAlert;

  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};

  public SwerveModule(SwerveModuleIO io, int index, SwerveModuleConstants constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;

    driveDisconnectedAlert =
        new Alert("Disconnected drive motor on swerve module " + index + ".", AlertType.kError);
    turnDisconnectedAlert =
        new Alert("Disconnected turn motor on swerve module " + index + ".", AlertType.kError);
    turnCanCoderDisconnectedAlert =
        new Alert("Disconnected turn CANcoder on swerve module " + index + ".", AlertType.kError);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Swerve/Module" + index, inputs);

    int sampleCount = inputs.odometryTimestamps.length;
    odometryPositions = new SwerveModulePosition[sampleCount];

    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnCanCoderDisconnectedAlert.set(!inputs.turnCanCoderConnected);
  }

  public void runSetpoint(SwerveModuleState state) {
    state.optimize(getAngle());
    state.cosineScale(inputs.turnAbsolutePosition);

    io.setDriveVelocity(state.speedMetersPerSecond / constants.WheelRadius);
    io.setTurnPosition(state.angle);
  }

  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(new Rotation2d());
  }

  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public Rotation2d getAngle() {
    return inputs.turnAbsolutePosition;
  }

  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  public double getVelocityMetersPerSecond() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSecond(), getAngle());
  }

  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
