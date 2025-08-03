package org.teamtitanium.frc2025.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.teamtitanium.frc2025.Constants;
import org.teamtitanium.frc2025.TunerConstants;
import org.teamtitanium.frc2025.utils.PhoenixUtil;

public class SwerveModuleIOTalonFX implements SwerveModuleIO {
  protected final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  protected final TalonFX driveMotor;
  protected final TalonFX turnMotor;
  protected final CANcoder canCoder;

  protected final VoltageOut voltageOut = new VoltageOut(0);
  protected final PositionVoltage positionVoltage = new PositionVoltage(0);
  protected final VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  protected final TorqueCurrentFOC torqueCurrentFOC = new TorqueCurrentFOC(0);
  protected final PositionTorqueCurrentFOC positionTorqueCurrentFOC =
      new PositionTorqueCurrentFOC(0);
  protected final VelocityTorqueCurrentFOC velocityTorqueCurrentFOC =
      new VelocityTorqueCurrentFOC(0);

  protected final StatusSignal<Angle> drivePosition;
  protected final StatusSignal<AngularVelocity> driveVelocity;
  protected final StatusSignal<Voltage> driveAppliedVoltage;
  protected final StatusSignal<Current> driveCurrent;
  protected final StatusSignal<Temperature> driveTemperature;

  protected final StatusSignal<Angle> turnAbsolutePosition;
  protected final StatusSignal<Angle> turnPosition;
  protected final StatusSignal<AngularVelocity> turnVelocity;
  protected final StatusSignal<Voltage> turnAppliedVoltage;
  protected final StatusSignal<Current> turnCurrent;
  protected final StatusSignal<Temperature> turnTemperature;

  private final Debouncer driveConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer turnConnectedDebouncer = new Debouncer(0.5);
  private final Debouncer turnCanCoderConnectedDebouncer = new Debouncer(0.5);

  protected SwerveModuleIOTalonFX(
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.constants = constants;

    driveMotor = new TalonFX(constants.DriveMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    turnMotor = new TalonFX(constants.SteerMotorId, TunerConstants.DrivetrainConstants.CANBusName);
    canCoder = new CANcoder(constants.EncoderId, TunerConstants.DrivetrainConstants.CANBusName);

    var driveConfig = constants.DriveMotorInitialConfigs;
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.Slot0 = constants.DriveMotorGains;
    driveConfig.Feedback.SensorToMechanismRatio = constants.DriveMotorGearRatio;
    driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = constants.SlipCurrent;
    driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimit = constants.SlipCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    driveConfig.MotorOutput.Inverted =
        constants.DriveMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> driveMotor.getConfigurator().apply(driveConfig, 0.25));
    PhoenixUtil.tryUntilOk(5, () -> driveMotor.setPosition(0.0, 0.25));

    var turnConfig = new TalonFXConfiguration();
    turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    turnConfig.Slot0 = constants.SteerMotorGains;
    if (Constants.getMode() == Constants.Mode.SIM) {
      turnConfig.Slot0.withKD(0.5).withKS(0.0);
    }

    turnConfig.Feedback.FeedbackRemoteSensorID = constants.EncoderId;
    turnConfig.Feedback.FeedbackSensorSource =
        switch (constants.FeedbackSource) {
          case RemoteCANcoder -> FeedbackSensorSourceValue.RemoteCANcoder;
          case FusedCANcoder -> FeedbackSensorSourceValue.FusedCANcoder;
          case SyncCANcoder -> FeedbackSensorSourceValue.SyncCANcoder;
          default -> FeedbackSensorSourceValue.RemoteCANcoder;
        };
    turnConfig.Feedback.RotorToSensorRatio = constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicCruiseVelocity = 100.0 / constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicAcceleration =
        turnConfig.MotionMagic.MotionMagicCruiseVelocity / 0.1;
    turnConfig.MotionMagic.MotionMagicExpo_kV = 0.12 * constants.SteerMotorGearRatio;
    turnConfig.MotionMagic.MotionMagicExpo_kA = 0.1;
    turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
    turnConfig.MotorOutput.Inverted =
        constants.SteerMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> turnMotor.getConfigurator().apply(turnConfig, 0.25));

    CANcoderConfiguration canCoderConfig = constants.EncoderInitialConfigs;
    canCoderConfig.MagnetSensor.MagnetOffset = constants.EncoderOffset;
    canCoderConfig.MagnetSensor.SensorDirection =
        constants.EncoderInverted
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> canCoder.getConfigurator().apply(canCoderConfig, 0.25));

    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVoltage = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();
    driveTemperature = driveMotor.getDeviceTemp();

    turnAbsolutePosition = canCoder.getPosition();
    turnPosition = turnMotor.getPosition();
    turnVelocity = turnMotor.getVelocity();
    turnAppliedVoltage = turnMotor.getMotorVoltage();
    turnCurrent = turnMotor.getStatorCurrent();
    turnTemperature = turnMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        Swerve.ODOMETRY_FREQUENCY, turnAbsolutePosition, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        driveVelocity,
        driveAppliedVoltage,
        driveCurrent,
        driveTemperature,
        turnPosition,
        turnVelocity,
        turnAppliedVoltage,
        turnCurrent,
        turnTemperature);
    ParentDevice.optimizeBusUtilizationForAll(driveMotor, turnMotor);
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    var driveStatus =
        BaseStatusSignal.refreshAll(
            drivePosition, driveVelocity, driveAppliedVoltage, driveCurrent, driveTemperature);
    var turnStatus =
        BaseStatusSignal.refreshAll(
            turnAbsolutePosition,
            turnPosition,
            turnVelocity,
            turnAppliedVoltage,
            turnCurrent,
            turnTemperature);
    var turnCanCoderStatus =
        BaseStatusSignal.refreshAll(canCoder.getPosition(), canCoder.getVelocity());

    inputs.driveConnected = driveConnectedDebouncer.calculate(driveStatus.isOK());
    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveAppliedVolts = driveAppliedVoltage.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    inputs.driveTempCelsius = driveTemperature.getValueAsDouble();

    inputs.turnConnected = turnConnectedDebouncer.calculate(turnStatus.isOK());
    inputs.turnCanCoderConnected =
        turnCanCoderConnectedDebouncer.calculate(turnCanCoderStatus.isOK());
    inputs.turnAbsolutePosition = Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnAppliedVolts = turnAppliedVoltage.getValueAsDouble();
    inputs.turnCurrentAmps = turnCurrent.getValueAsDouble();
    inputs.turnTempCelsius = turnTemperature.getValueAsDouble();
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveMotor.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> voltageOut.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentFOC.withOutput(output);
        });
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnMotor.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> voltageOut.withOutput(output);
          case TorqueCurrentFOC -> torqueCurrentFOC.withOutput(output);
        });
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveMotor.setControl(
        switch (constants.DriveMotorClosedLoopOutput) {
          case Voltage -> velocityVoltage.withVelocity(Units.radiansToRotations(velocityRadPerSec));
          case TorqueCurrentFOC ->
              velocityTorqueCurrentFOC.withVelocity(Units.radiansToRotations(velocityRadPerSec));
        });
  }

  @Override
  public void setTurnPosition(Rotation2d position) {
    turnMotor.setControl(
        switch (constants.SteerMotorClosedLoopOutput) {
          case Voltage -> positionVoltage.withPosition(position.getRotations());
          case TorqueCurrentFOC -> positionTorqueCurrentFOC.withPosition(position.getRotations());
        });
  }
}
