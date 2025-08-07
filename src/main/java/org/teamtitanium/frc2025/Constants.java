package org.teamtitanium.frc2025;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;
  public static final boolean disableHAL = false;
  private static RobotType robotType = RobotType.SIMBOT;

  @SuppressWarnings("resource")
  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid robot selected, using competition robot as default.", AlertType.kError);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (robotType) {
      case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    REAL,
    SIM,
    REPLAY
  }

  public enum RobotType {
    COMPBOT,
    SIMBOT
  }

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kG, double kA) {
    public Gains(double kP, double kI, double kD) {
      this(kP, kI, kD, 0.0, 0.0, 0.0, 0.0);
    }

    public Gains(double kP, double kI, double kD, double kS, double kV, double kG) {
      this(kP, kI, kD, kS, kV, kG, 0.0);
    }
  }

  public record Constraints(double maxVelocity, double maxAcceleration, double jerk) {
    public Constraints(double maxVelocity, double maxAcceleration) {
      this(maxVelocity, maxAcceleration, 0.0);
    }
  }
}
