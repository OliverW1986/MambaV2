package org.teamtitanium.frc2025.subsystems.elevator;

import java.util.List;

import org.teamtitanium.frc2025.utils.PhoenixUtil;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static org.teamtitanium.frc2025.subsystems.elevator.ElevatorConstants.*;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX leaderMotor, followerMotor;

    private final TalonFXConfiguration config = new TalonFXConfiguration();

    private final MotionMagicVoltage motionMagicVoltage = new MotionMagicVoltage(0.0);
    private final VoltageOut voltageOut = new VoltageOut(0.0);

    private final StatusSignal<Double> elevatorSetpoint;

    private final StatusSignal<Angle> elevatorPosition;
    private final StatusSignal<AngularVelocity> elevatorVelocity;
    private final List<StatusSignal<Voltage>> appliedVolts;
    private final List<StatusSignal<Current>> supplyCurrentAmps;
    private final List<StatusSignal<Current>> torqueCurrentAmps;
    private final List<StatusSignal<Temperature>> tempCelsius;

    public ElevatorIOTalonFX() {
        leaderMotor = new TalonFX(LEADER_MOTOR_ID, ELEVATOR_CANBUS);
        followerMotor = new TalonFX(FOLLOWER_MOTOR_ID, ELEVATOR_CANBUS);

        config.Slot0.kP = ELEVATOR_GAINS.kP();
        config.Slot0.kI = ELEVATOR_GAINS.kI();
        config.Slot0.kD = ELEVATOR_GAINS.kD();
        config.Slot0.kS = ELEVATOR_GAINS.kS();
        config.Slot0.kV = ELEVATOR_GAINS.kV();
        config.Slot0.kG = ELEVATOR_GAINS.kG();
        config.Slot0.kA = ELEVATOR_GAINS.kA();

        config.MotionMagic.MotionMagicCruiseVelocity = ELEVATOR_CONSTRAINTS.maxVelocity();
        config.MotionMagic.MotionMagicAcceleration = ELEVATOR_CONSTRAINTS.maxAcceleration();

        config.CurrentLimits.StatorCurrentLimit = STATOR_CURRENT_LIMIT;
        config.CurrentLimits.StatorCurrentLimitEnable = true;

        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;

        config.MotorOutput.Inverted = ELEVATOR_INVERTED ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.Feedback.SensorToMechanismRatio = GEAR_RATIO;

        PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config, 0.25));
        PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config, 0.25));
        followerMotor.setControl(new Follower(LEADER_MOTOR_ID, true));

        elevatorSetpoint = leaderMotor.getClosedLoopReference();

        elevatorPosition = leaderMotor.getPosition();
        elevatorVelocity = leaderMotor.getVelocity();
        appliedVolts = List.of(leaderMotor.getMotorVoltage(), followerMotor.getMotorVoltage());
        supplyCurrentAmps = List.of(leaderMotor.getSupplyCurrent(), followerMotor.getSupplyCurrent());
        torqueCurrentAmps = List.of(leaderMotor.getTorqueCurrent(), followerMotor.getTorqueCurrent());
        tempCelsius = List.of(leaderMotor.getDeviceTemp(), followerMotor.getDeviceTemp());

        BaseStatusSignal.setUpdateFrequencyForAll(250, elevatorPosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                100,
                elevatorVelocity,
                appliedVolts.get(0),
                appliedVolts.get(1),
                supplyCurrentAmps.get(0),
                supplyCurrentAmps.get(1),
                torqueCurrentAmps.get(0),
                torqueCurrentAmps.get(1),
                tempCelsius.get(0),
                tempCelsius.get(1),
                elevatorSetpoint);

        leaderMotor.optimizeBusUtilization();
        followerMotor.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leaderConnected = BaseStatusSignal.refreshAll(
                elevatorPosition,
                elevatorVelocity,
                appliedVolts.get(0),
                supplyCurrentAmps.get(0),
                torqueCurrentAmps.get(0),
                tempCelsius.get(0),
                elevatorSetpoint)
                .isOK();
        inputs.followerConnected = BaseStatusSignal.refreshAll(
                appliedVolts.get(1),
                supplyCurrentAmps.get(1),
                torqueCurrentAmps.get(1),
                tempCelsius.get(1))
                .isOK();

        inputs.positionSetpoint = elevatorSetpoint.getValue();

        inputs.positionRots = elevatorPosition.getValue().in(Rotations);
        inputs.velocityRps = elevatorVelocity.getValue().in(RotationsPerSecond);
        inputs.appliedVolts =
            appliedVolts.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.supplyCurrentAmps =
            supplyCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.torqueCurrentAmps =
            torqueCurrentAmps.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
        inputs.tempCelsius = tempCelsius.stream().mapToDouble(StatusSignal::getValueAsDouble).toArray();
    }
}
