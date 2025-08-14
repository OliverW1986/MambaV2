// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.teamtitanium.frc2025;

import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.HashMap;
import java.util.function.BiConsumer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.teamtitanium.frc2025.Constants.RobotType;
import org.teamtitanium.frc2025.commands.DriveCommands;
import org.teamtitanium.frc2025.subsystems.elevator.Elevator;
import org.teamtitanium.frc2025.subsystems.elevator.ElevatorIO;
import org.teamtitanium.frc2025.subsystems.elevator.ElevatorIOSim;
import org.teamtitanium.frc2025.subsystems.elevator.ElevatorIOTalonFX;
import org.teamtitanium.frc2025.subsystems.swerve.GyroIO;
import org.teamtitanium.frc2025.subsystems.swerve.GyroIOPigeon2;
import org.teamtitanium.frc2025.subsystems.swerve.GyroIOSim;
import org.teamtitanium.frc2025.subsystems.swerve.Swerve;
import org.teamtitanium.frc2025.subsystems.swerve.SwerveModuleIO;
import org.teamtitanium.frc2025.subsystems.swerve.SwerveModuleIOTalonFXReal;
import org.teamtitanium.frc2025.subsystems.swerve.SwerveModuleIOTalonFXSim;

public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final Swerve swerve;
    private final Elevator elevator;

    private SwerveDriveSimulation swerveDriveSimulation = null;

    private final CommandXboxController driver = new CommandXboxController(0);

    public Robot() {
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("RuntimeType", getRuntimeType().toString());
        Logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes present");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown dirty state");
                break;
        }

        switch (Constants.getMode()) {
            case REAL:
                Logger.addDataReceiver(new WPILOGWriter());
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case SIM:
                Logger.addDataReceiver(new NT4Publisher());
                break;
            case REPLAY:
                setUseTiming(false);
                var logPath = LogFileUtil.findReplayLog();
                Logger.setReplaySource(new WPILOGReader(logPath));
                Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
                break;
        }

        Logger.start();

        SignalLogger.enableAutoLogging(false);

        DriverStation.silenceJoystickConnectionWarning(true);

        var commandCounts = new HashMap<String, Integer>();
        BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
            var name = command.getName();
            var count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput(
                    "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
        CommandScheduler.getInstance()
                .onCommandInitialize((Command command) -> logCommandFunction.accept(command, true));
        CommandScheduler.getInstance()
                .onCommandFinish((Command command) -> logCommandFunction.accept(command, false));
        CommandScheduler.getInstance()
                .onCommandInterrupt((Command command) -> logCommandFunction.accept(command, false));

        if (Constants.getRobot() == RobotType.SIMBOT) {
            DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
            DriverStationSim.notifyNewData();
        }

        switch (Constants.getMode()) {
            case REAL:
                swerve = new Swerve(
                        new GyroIOPigeon2(),
                        new SwerveModuleIOTalonFXReal(TunerConstants.FrontLeft),
                        new SwerveModuleIOTalonFXReal(TunerConstants.FrontRight),
                        new SwerveModuleIOTalonFXReal(TunerConstants.BackLeft),
                        new SwerveModuleIOTalonFXReal(TunerConstants.BackRight),
                        (pose) -> {
                        });
                elevator = new Elevator(new ElevatorIOTalonFX());
                break;
            case SIM:
                swerveDriveSimulation = new SwerveDriveSimulation(Swerve.mapleSimConfig,
                        new Pose2d(3, 3, new Rotation2d()));
                SimulatedArena.getInstance().addDriveTrainSimulation(swerveDriveSimulation);
                swerve = new Swerve(
                        new GyroIOSim(swerveDriveSimulation.getGyroSimulation()),
                        new SwerveModuleIOTalonFXSim(
                                TunerConstants.FrontLeft, swerveDriveSimulation.getModules()[0]),
                        new SwerveModuleIOTalonFXSim(
                                TunerConstants.FrontRight, swerveDriveSimulation.getModules()[1]),
                        new SwerveModuleIOTalonFXSim(
                                TunerConstants.BackLeft, swerveDriveSimulation.getModules()[2]),
                        new SwerveModuleIOTalonFXSim(
                                TunerConstants.BackRight, swerveDriveSimulation.getModules()[3]),
                        swerveDriveSimulation::setSimulationWorldPose);
                elevator = new Elevator(new ElevatorIOSim());
                break;
            default:
                swerve = new Swerve(
                        new GyroIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        new SwerveModuleIO() {
                        },
                        (pose) -> {
                        });
                elevator = new Elevator(new ElevatorIO() {

                });
                break;
        }

        swerve.setDefaultCommand(
                DriveCommands.joystickDrive(
                        swerve, () -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX()));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        if (Constants.getMode() != Constants.Mode.SIM) {
            return;
        }

        swerveDriveSimulation.setSimulationWorldPose(new Pose2d(3, 3, new Rotation2d()));
        SimulatedArena.getInstance().resetFieldForAuto();
        swerve.setPose(new Pose2d(3, 3, new Rotation2d()));
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {
        if (Constants.getMode() != Constants.Mode.SIM) {
            return;
        }

        SimulatedArena.getInstance().simulationPeriodic();
        Logger.recordOutput(
                "FieldSimulation/RobotPosition", swerveDriveSimulation.getSimulatedDriveTrainPose());
        Logger.recordOutput(
                "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
        Logger.recordOutput(
                "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
    }
}
