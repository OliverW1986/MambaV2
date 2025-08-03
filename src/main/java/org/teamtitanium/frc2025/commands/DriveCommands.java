package org.teamtitanium.frc2025.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.function.DoubleSupplier;
import org.teamtitanium.frc2025.subsystems.swerve.Swerve;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

    linearMagnitude = linearMagnitude * linearMagnitude;

    return new Pose2d(new Translation2d(), linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
        .getTranslation();
  }

  public static Command joystickDrive(
      Swerve swerve,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier thetaSupplier) {
    return Commands.run(
        () -> {
          Translation2d linearVelocity =
              getLinearVelocityFromJoysticks(xSupplier.getAsDouble(), ySupplier.getAsDouble());

          double theta = MathUtil.applyDeadband(thetaSupplier.getAsDouble(), DEADBAND);

          theta = Math.copySign(theta * theta, theta);

          ChassisSpeeds speeds =
              new ChassisSpeeds(
                  linearVelocity.getX() * swerve.getMaxLinearSpeedMetersPerSec(),
                  linearVelocity.getY() * swerve.getMaxLinearSpeedMetersPerSec(),
                  theta * swerve.getMaxAngularSpeedRadPerSec());
          boolean isFlipped =
              DriverStation.getAlliance().isPresent()
                  && DriverStation.getAlliance().get() == DriverStation.Alliance.Red;
          swerve.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  speeds,
                  isFlipped ? swerve.getRotation().plus(Rotation2d.kPi) : swerve.getRotation()));
        },
        swerve);
  }
}
