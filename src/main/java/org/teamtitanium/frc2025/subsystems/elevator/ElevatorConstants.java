package org.teamtitanium.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import org.teamtitanium.frc2025.Constants;
import org.teamtitanium.frc2025.Constants.Constraints;
import org.teamtitanium.frc2025.Constants.Gains;
import org.teamtitanium.frc2025.utils.LoggedTunableNumber;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static final LoggedTunableNumber TUNABLE_ELEVATOR_HEIGHT =
      new LoggedTunableNumber("Elevator/TunableHeight", 0.0);
  public static final LoggedTunableNumber TUNABLE_ELEVATOR_HEIGHT_2 =
      new LoggedTunableNumber("Elevator/TunableHeight2", 0.0);

  // Setpoint Constants
  public static final Distance L1_HEIGHT = Inches.of(5);
  public static final Distance L2_HEIGHT = Inches.of(18.0);
  public static final Distance L3_HEIGHT = Inches.of(32.5);
  public static final Distance L4_HEIGHT = Inches.of(58.5);

  public static final Distance INTAKE_ALGAE_HIGH_FIRST = Inches.of(20);
  public static final Distance INTAKE_ALGAE_HIGH_SECOND = Inches.of(28);

  public static final Distance INTAKE_ALGAE_MIDDLE_FIRST = Inches.of(4.5);
  public static final Distance INTAKE_ALGAE_MIDDLE_SECOND = Inches.of(12.5);

  public static final Distance INTAKE_ALGAE_STACK = Inches.of(16);
  public static final Distance INTAKE_ALGAE_GOUND = Inches.of(3.0);

  public static final Distance GROUND_INTAKE_DEPLOY_HEIGHT = Inches.of(16);
  public static final Distance GROUND_INTAKE_HANDOFF_HEIGHT = Inches.of(16);

  public static final Distance NET_HEIGHT = Inches.of(73);
  public static final Distance PROCESSOR_HEIGHT = Inches.of(7);

  // Physical Constants
  public static final int LEADER_MOTOR_ID = 12;
  public static final int FOLLOWER_MOTOR_ID = 13;

  public static final String ELEVATOR_CANBUS = "Canivore";

  public static final double GEAR_RATIO = 8.55;

  public static final double STATOR_CURRENT_LIMIT = 60;

  public static final boolean ELEVATOR_INVERTED = false;

  public static final Distance ELEVATOR_MAX_HEIGHT = Inches.of(75.5);
  public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(0);

  public static final Gains ELEVATOR_GAINS =
      switch (Constants.getMode()) {
        case REAL -> new Gains(12, 0, 0, 0.17125, 1.25, 0.30875, 0.02);
        case SIM -> new Gains(50.0, 0, 0, 0.0, 1.15, 0.25, 0.0);
        case REPLAY -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
      };
  public static final Constraints ELEVATOR_CONSTRAINTS = new Constraints(10.5, 30.0);

  public static final Pose3d[] SIM_ELEVATOR_COMPONENT_OFFSETS =
      new Pose3d[] {
        new Pose3d(0, 0, 0.09675, new Rotation3d()),
        new Pose3d(0, 0, 0.12225, new Rotation3d()),
        new Pose3d(0, 0, 0.147, new Rotation3d())
      };

  public static final double SIM_STAGE_1_MAX_HEIGHT = 0.743;
  public static final double SIM_STAGE_2_MAX_HEIGHT = 1.455;

  public static final double SIM_STAGE_1_MOVEMENT_HEIGHT = 1.274;
  public static final double SIM_STAGE_2_MOVEMENT_HEIGHT = 0.584;
}
