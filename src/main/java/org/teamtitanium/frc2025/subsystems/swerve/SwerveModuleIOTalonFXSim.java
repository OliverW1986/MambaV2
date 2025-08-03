package org.teamtitanium.frc2025.subsystems.swerve;

import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import java.util.Arrays;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.teamtitanium.frc2025.utils.PhoenixUtil;

public class SwerveModuleIOTalonFXSim extends SwerveModuleIOTalonFX {
  private final SwerveModuleSimulation simulation;

  @SuppressWarnings({"rawtypes", "unchecked"})
  public SwerveModuleIOTalonFXSim(
      SwerveModuleConstants constants, SwerveModuleSimulation simulation) {
    super(PhoenixUtil.regulateModuleConstantsForSimulation(constants));

    this.simulation = simulation;
    simulation.useDriveMotorController(new PhoenixUtil.TalonFXMotorControllerSim(driveMotor));

    simulation.useSteerMotorController(
        new PhoenixUtil.TalonFXMotorControllerWithRemoteCancoderSim(turnMotor, canCoder));
  }

  @Override
  public void updateInputs(SwerveModuleIOInputs inputs) {
    super.updateInputs(inputs);

    inputs.odometryTimestamps = PhoenixUtil.getSimulationOdometryTimestamps();

    inputs.odometryDrivePositionsRad =
        Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
            .mapToDouble(angle -> angle.in(Radians))
            .toArray();

    inputs.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
  }
}
