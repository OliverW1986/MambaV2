package org.teamtitanium.frc2025.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;

import org.teamtitanium.frc2025.Constants;

import com.ctre.phoenix6.BaseStatusSignal;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim extends ElevatorIOTalonFX {
    private final ElevatorSim sim = new ElevatorSim(DCMotor.getKrakenX60Foc(2), ElevatorConstants.GEAR_RATIO,
            Units.lbsToKilograms(20), Units.inchesToMeters(1.1), 0, ElevatorConstants.ELEVATOR_MAX_HEIGHT.in(Meters),
            true, 0);

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

        var leaderMotorSim = leaderMotor.getSimState();

        leaderMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        sim.setInputVoltage(leaderMotorSim.getMotorVoltage());
        sim.update(Constants.loopPeriodSecs);

        leaderMotorSim.setRawRotorPosition(sim.getPositionMeters() / (2 * Math.PI * Units.inchesToMeters(1.1))
                * ElevatorConstants.GEAR_RATIO);
        leaderMotorSim.setRotorVelocity(sim.getVelocityMetersPerSecond() / (2 * Math.PI * Units.inchesToMeters(1.1))
                * ElevatorConstants.GEAR_RATIO);

        super.updateInputs(inputs);
    }
}
