package org.teamtitanium.frc2025;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotBase;

public class Constants {
  public static final double loopPeriodSecs = 0.02;
  public static final boolean tuningMode = false;
  private static RobotType robotType = RobotType.COMPBOT;

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

  public static class ChechDeploy {}
}
