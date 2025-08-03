package org.teamtitanium.frc2025.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.frc2025.Constants.Gains;

public interface SwerveModuleIO {
  @AutoLog
  public class SwerveModuleIOInputs {
    public boolean driveConnected = false;
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    public double driveTempCelsius = 0.0;

    public boolean turnConnected = false;
    public boolean turnCanCoderConnected = false;
    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double turnCurrentAmps = 0.0;
    public double turnTempCelsius = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  public default void updateInputs(SwerveModuleIOInputs inputs) {}

  public default void setDriveOpenLoop(double output) {}

  public default void setTurnOpenLoop(double output) {}

  public default void setDriveVelocity(double velocityRadPerSec) {}

  public default void setTurnPosition(Rotation2d positionRad) {}

  public default void setDriveGains(Gains gains) {}

  public default void setTurnGains(Gains gains) {}

  public default void setBrakeMode(boolean enabled) {}
}
