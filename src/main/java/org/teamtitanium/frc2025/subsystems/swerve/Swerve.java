package org.teamtitanium.frc2025.subsystems.swerve;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.ironmaple.simulation.drivesims.configs.SwerveModuleSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.teamtitanium.frc2025.Constants;
import org.teamtitanium.frc2025.Constants.Mode;
import org.teamtitanium.frc2025.TunerConstants;

public class Swerve extends SubsystemBase {
  public static final double ODOMETRY_FREQUENCY =
      new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
              Math.hypot(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
              Math.hypot(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)));

  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;

  public static final DriveTrainSimulationConfig mapleSimConfig =
      DriveTrainSimulationConfig.Default()
          .withRobotMass(Kilograms.of(ROBOT_MASS_KG))
          .withCustomModuleTranslations(getSwerveModuleTranslations())
          .withGyro(COTS.ofPigeon2())
          .withSwerveModule(
              new SwerveModuleSimulationConfig(
                  DCMotor.getKrakenX60(1),
                  DCMotor.getKrakenX60(1),
                  TunerConstants.FrontLeft.DriveMotorGearRatio,
                  TunerConstants.FrontLeft.SteerMotorGearRatio,
                  Volts.of(TunerConstants.FrontLeft.DriveFrictionVoltage),
                  Volts.of(TunerConstants.FrontLeft.SteerFrictionVoltage),
                  Meters.of(TunerConstants.FrontLeft.WheelRadius),
                  KilogramSquareMeters.of(TunerConstants.FrontLeft.SteerInertia),
                  WHEEL_COF));

  public static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroIOInputs = new GyroIOInputsAutoLogged();
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
  private final SwerveModule[] swerveModules = new SwerveModule[4];

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(getSwerveModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions =
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private final SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private final Consumer<Pose2d> resetSimulationPoseCallBack;

  private final SysIdRoutine sysId;

  public Swerve(
      GyroIO gyroIO,
      SwerveModuleIO flModuleIO,
      SwerveModuleIO frModuleIO,
      SwerveModuleIO blModuleIO,
      SwerveModuleIO brModuleIO,
      Consumer<Pose2d> resetSimulationPoseCallBack) {
    this.gyroIO = gyroIO;
    this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

    swerveModules[0] = new SwerveModule(flModuleIO, 0, TunerConstants.FrontLeft);
    swerveModules[1] = new SwerveModule(frModuleIO, 1, TunerConstants.FrontRight);
    swerveModules[2] = new SwerveModule(blModuleIO, 2, TunerConstants.BackLeft);
    swerveModules[3] = new SwerveModule(brModuleIO, 3, TunerConstants.BackRight);

    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    PhoenixOdometryThread.getInstance().start();

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Swerve/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
  }

  @Override
  public void periodic() {
    odometryLock.lock();
    gyroIO.updateInputs(gyroIOInputs);
    Logger.processInputs("Swerve/Gyro", gyroIOInputs);

    for (SwerveModule swerveModule : swerveModules) {
      swerveModule.periodic();
    }
    odometryLock.unlock();

    if (DriverStation.isDisabled()) {
      for (SwerveModule module : swerveModules) {
        module.stop();
      }
    }

    if (DriverStation.isDisabled()) {
      Logger.recordOutput("Swerve/SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    double[] sampleTimestamps = swerveModules[0].getOdometryTimestamps();
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = swerveModules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      if (gyroIOInputs.connected) {
        rawGyroRotation = gyroIOInputs.odometryYawPositions[i];
      } else {
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    gyroDisconnectedAlert.set(!gyroIOInputs.connected && Constants.getMode() != Mode.SIM);
  }

  public void runVelocity(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, Constants.loopPeriodSecs);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

    Logger.recordOutput("Swerve/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Swerve/SwerveChassisSpeeds/Setpoints", speeds);

    for (int i = 0; i < swerveModules.length; i++) {
      swerveModules[i].runSetpoint(setpointStates[i]);
    }

    Logger.recordOutput("Swerve/SwerveStates/SetpointsOptimized", setpointStates);
  }

  private void runCharacterization(double output) {
    for (SwerveModule module : swerveModules) {
      module.runCharacterization(output);
    }
  }

  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getSwerveModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction))
        .finallyDo(() -> stop());
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.dynamic(direction))
        .finallyDo(() -> stop());
  }

  @AutoLogOutput(key = "Swerve/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < swerveModules.length; i++) {
      states[i] = swerveModules[i].getState();
    }
    return states;
  }

  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    for (int i = 0; i < swerveModules.length; i++) {
      positions[i] = swerveModules[i].getPosition();
    }
    return positions;
  }

  @AutoLogOutput(key = "Swerve/SwerveChassisSpeeds/Measured")
  private ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  public double[] getWheelRadiusCharacterizationPositions() {
    double[] positions = new double[4];
    for (int i = 0; i < 4; i++) {
      positions[i] = swerveModules[i].getWheelRadiusCharacterizationPosition();
    }
    return positions;
  }

  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (SwerveModule swerveModule : swerveModules) {
      output += swerveModule.getFFCharacterizationVelocity();
    }
    output /= swerveModules.length;
    return output;
  }

  @AutoLogOutput(key = "Odometry/RobotPose")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public void setPose(Pose2d pose) {
    resetSimulationPoseCallBack.accept(pose);
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  public double getMaxLinearSpeedMetersPerSec() {
    return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  public static Translation2d[] getSwerveModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
    };
  }
}
