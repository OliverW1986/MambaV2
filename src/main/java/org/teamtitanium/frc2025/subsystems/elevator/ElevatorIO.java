package org.teamtitanium.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;
import org.teamtitanium.frc2025.Constants.Constraints;
import org.teamtitanium.frc2025.Constants.Gains;

public interface ElevatorIO {
    @AutoLog
    public class ElevatorIOInputs {
        public boolean leaderConnected = false;
        public boolean followerConnected = false;

        public double positionSetpoint = 0.0;
        
        public double positionRots = 0.0;
        public double velocityRps = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    public default void updateInputs(ElevatorIOInputs inputs) {}

    public default void setPosition(double positionRots) {}

    public default void setVoltage(double voltage) {}

    public default void setGains(Gains gains) {}

    public default void setConstraints(Constraints constraints) {}

    public default void setBrakeMode(boolean enabled) {}

    public default void setMotorPosition(double positionRots) {}
}
