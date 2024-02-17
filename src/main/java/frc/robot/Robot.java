// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.rmi.server.ServerCloneException;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.autonomous.Autonomous;
import frc.autonomous.AutonomousProgram;
import frc.commands.DefaultDrive;
import frc.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.commands.DriveWait;
import frc.commands.TurnToSpeaker;
import frc.subsystems.DriveSubsystem;
import frc.modules.PhotonvisionModule;

public class Robot extends LoggedRobot {
	public static final CommandXboxController controller = new CommandXboxController(0);
	public static final AHRS navX = new AHRS();
	public static PhotonvisionModule photonvision;
	public static DriveSubsystem swerve;
	// public static Logger logger;

	@Override
	public void robotInit() {
		photonvision = null; //new PhotonvisionModule();
		swerve = new DriveSubsystem();
		swerve.resetOdometry();

		NamedCommands.registerCommand("PickUpNote", new InstantCommand(() -> {System.out.println("picked up note!");}));
		NamedCommands.registerCommand("TurnToSpeaker", new TurnToSpeaker());
		NamedCommands.registerCommand("Shooting", new InstantCommand(() -> {System.out.println("Shooting note!");}));

		configureButtonBindings();

		Logger.recordMetadata("Java-Command-2024", "robot"); // Set a metadata value

		if (isReal()) {
			// Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			new PowerDistribution(13, ModuleType.kRev); // Enables power distribution logging
		} else {
			// setUseTiming(false); // Run as fast as possible
			// String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the
			// 												// user)
			// Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			// Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a
			Logger.addDataReceiver(new NT4Publisher());																	// new log
		}

		Logger.start();



		Autonomous.init();
		AutonomousProgram.addAutosToShuffleboard();
	}

	@Override
	public void simulationInit() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		controller.y().onTrue(new InstantCommand(swerve::resetOdometry));
		controller.b().onTrue(new InstantCommand(swerve::resetOdometryWidget));
		controller.a().onTrue(new TurnToSpeaker());
	}

	/* Currently running auto routine */

	private Command autonomousCommand;

	@Override
	public void autonomousInit() {
		navX.reset();
		Command autonomousRoutine = AutonomousProgram.constructSelectedRoutine();

		// Home the arm while waiting for the drivebase delay
		Command delay = new ParallelCommandGroup(new DriveWait(AutonomousProgram.getAutonomousDelay()));

		// Schedule the selected autonomous command group
		if (autonomousRoutine != null) {
			// Run the delay/home and the selected routine sequentially
			this.autonomousCommand = new SequentialCommandGroup(
					delay,
					autonomousRoutine);
		} else {
			this.autonomousCommand = delay;
		}

		// Schedule the combined command group
		CommandScheduler.getInstance().schedule(this.autonomousCommand);
	}

	@Override
	public void disabledInit() {

		// run resetEncoder counter in case we get a bad reading
		swerve.robotRelativeDrive(new ChassisSpeeds());
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}
    	swerve.setDefaultCommand(new DefaultDrive(false));
	}

	@Override
	public void robotPeriodic() { 

		CommandScheduler.getInstance().run();

	}
}
