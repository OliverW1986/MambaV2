package org.teamtitanium.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.Logger;
import org.teamtitanium.frc2025.Constants.Constraints;
import org.teamtitanium.frc2025.Constants.Gains;
import org.teamtitanium.frc2025.utils.LoggedTunableNumber;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Meters;
import static org.teamtitanium.frc2025.subsystems.elevator.ElevatorConstants.*;

import java.util.function.Supplier;

public class Elevator extends SubsystemBase {
    private final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", ELEVATOR_GAINS.kP());
    private final LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/kI", ELEVATOR_GAINS.kI());
    private final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", ELEVATOR_GAINS.kD());
    private final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", ELEVATOR_GAINS.kS());
    private final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", ELEVATOR_GAINS.kV());
    private final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", ELEVATOR_GAINS.kG());
    private final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", ELEVATOR_GAINS.kA());

    private final LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Elevator/maxVelocity",
            ELEVATOR_CONSTRAINTS.maxVelocity());
    private final LoggedTunableNumber maxAcceleration = new LoggedTunableNumber("Elevator/maxAcceleration",
            ELEVATOR_CONSTRAINTS.maxAcceleration());

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (kP.hasChanged(hashCode()) || kI.hasChanged(hashCode()) || kD.hasChanged(hashCode())
                || kS.hasChanged(hashCode()) || kV.hasChanged(hashCode()) || kG.hasChanged(hashCode())
                || kA.hasChanged(hashCode())) {
            io.setGains(new Gains(kP.get(), kI.get(), kD.get(), kS.get(), kV.get(), kG.get(), kA.get()));
        }

        if (maxVelocity.hasChanged(hashCode()) || maxAcceleration.hasChanged(hashCode())) {
            io.setConstraints(
                    new Constraints(maxVelocity.get(), maxAcceleration.get()));
        }
    }

    public Command setPosition(Supplier<Distance> position) {
        return run(() -> io.setPosition(position.get().in(Meters) / (2 * Math.PI * Units.inchesToMeters(1.1))));
    }

    public Command setPosition(Distance position) {
        return setPosition(() -> position);
    }

    public Command setTorqueCurrent(Supplier<Double> current) {
        return run(() -> io.setTorqueCurrent(current.get()));
    }

    public Command setTorqueCurrent(double current) {
        return setTorqueCurrent(() -> current);
    }

    public Command setVoltage(Supplier<Double> voltage) {
        return run(() -> io.setTorqueCurrent(voltage.get()));
    }

    public Command setVoltage(double voltage) {
        return setVoltage(() -> voltage);
    }

    public Command zeroElevator() {
        return setVoltage(-2).until(currentTrigger()).andThen(runOnce(() -> io.setVoltage(0.0)))
                .andThen(runOnce(() -> io.setMotorPosition(0.0)));
    }

    private Trigger currentTrigger() {
        return new Trigger(() -> Math.abs(inputs.torqueCurrentAmps[0]) >= 40).debounce(0.1);
    }

    public Trigger atHeight(Supplier<Distance> position) {
        return new Trigger(() -> Math.abs(inputs.positionRots - position.get().in(Meters)
                / (2 * Math.PI * Units.inchesToMeters(1.1))) < 0.2);
    }

    public Trigger atHeight(Distance position) {
        return atHeight(() -> position);
    }
}
