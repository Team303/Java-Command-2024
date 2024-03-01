// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.autonomous.Autonomous;
import frc.autonomous.AutonomousProgram;
import frc.commands.amoghbelt.IntakeNote;
import frc.commands.amoghbelt.ShootNote;
import frc.commands.drive.DefaultDrive;
import frc.commands.drive.DriveWait;
import frc.commands.drive.TurnToAngle;
import frc.commands.drive.TurnToSpeaker;
import frc.commands.intake.HomeIntake;
import frc.commands.intake.OutwardIntake;
import frc.commands.shooter.HomeShooter;
import frc.commands.shooter.ManualShootSpeaker;
import frc.commands.shooter.SetShooterAmp;
import frc.modules.PhotonvisionModule;
import frc.subsystems.Belt;
import frc.subsystems.DriveSubsystem;
import frc.subsystems.Intake;
import frc.subsystems.Shooter;
import frc.commands.intake.GroundIntake;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;

public class Robot extends LoggedRobot {
	public static final CommandXboxController driverController = new CommandXboxController(0);
	public static final CommandXboxController operatorController = new CommandXboxController(1);

	public static final AHRS navX = new AHRS();
	public static PhotonvisionModule photonvision;
	public static DriveSubsystem swerve;
	public static Intake intake;
	public static Belt belt;
	public static Shooter shooter;
	// public static Logger logger;

	@Override
	public void robotInit() {
		photonvision = null; // new PhotonvisionModule();
		swerve = new DriveSubsystem();
		intake = new Intake();
		belt = new Belt();
		shooter = new Shooter();
		swerve.resetOdometry();

		NamedCommands.registerCommand("PickUpNote", new InstantCommand(() -> {
			System.out.println("picked up note!");
		}));
		NamedCommands.registerCommand("TurnToSpeaker", new TurnToSpeaker());
		NamedCommands.registerCommand("Shooting", new InstantCommand(() -> {
			System.out.println("Shooting note!");
		}));

		configureButtonBindings();

		Logger.recordMetadata("Java-Command-2024", "robot"); // Set a metadata value

		if (isReal()) {
			// Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
			Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
			new PowerDistribution(13, ModuleType.kRev); // Enables power distribution logging
		} else {
			// setUseTiming(false); // Run as fast as possible
			// String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from
			// AdvantageScope (or prompt the
			// // user)
			// Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
			// Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath,
			// "_sim"))); // Save outputs to a
			Logger.addDataReceiver(new NT4Publisher()); // new log
		}

		Logger.start();

		Autonomous.init();
		AutonomousProgram.addAutosToShuffleboard();
	}

	@Override
	public void disabledInit() {
		// swerve.periodicReset();
	}

	@Override
	public void simulationInit() {
		configureButtonBindings();
	}

	private void configureButtonBindings() {
		driverController.y().onTrue(new InstantCommand(swerve::resetOdometry));
		driverController.a().toggleOnTrue(new TurnToAngle(0).repeatedly());

		// for testing only pls
		// operatorController.b().onTrue(new
		// InstantCommand(swerve::resetOdometryWidget));

		operatorController.pov(180).toggleOnTrue(new SequentialCommandGroup(new
		GroundIntake(),
		new ParallelDeadlineGroup(new IntakeNote(), new
		GroundIntake().repeatedly())));

		operatorController.pov(180).toggleOnTrue(new IntakeNote());

		// operatorController.pov(0).whileTrue(new ShootNote());
		operatorController.y().toggleOnTrue(new SequentialCommandGroup(new OutwardIntake(),
				new ParallelCommandGroup(new OutwardIntake().repeatedly(),
						new SequentialCommandGroup(new SetShooterAmp(Math.toRadians(50), 17),
								new ParallelCommandGroup(new ShootNote(),
										new SetShooterAmp(Math.toRadians(50), 17).repeatedly())))));

		operatorController.a().toggleOnTrue(new SequentialCommandGroup(new OutwardIntake(),
				new ParallelCommandGroup(new OutwardIntake().repeatedly(),
						new SequentialCommandGroup(new SetShooterAmp(Math.toRadians(30), 17),
								new ParallelCommandGroup(new ShootNote(),
										new SetShooterAmp(Math.toRadians(30), 17).repeatedly())))));

		operatorController.rightBumper().toggleOnTrue(new SequentialCommandGroup(new OutwardIntake(),
				new ParallelCommandGroup(new OutwardIntake().repeatedly(), new SequentialCommandGroup(
						new SetShooterAmp(Math.toRadians(30), 17),
						new ParallelCommandGroup(new ShootNote(), new ManualShootSpeaker(4.064).repeatedly())))));

		operatorController.leftBumper().toggleOnTrue(new SetShooterAmp(Math.toRadians(50), -10));

		// operatorController.rightBumper().onTrue(new ManualShootSpeaker(4.064));

		// operatorController.a().toggleOnTrue(new SequentialCommandGroup(new
		// OutwardIntake(),
		// new ParallelCommandGroup(new OutwardIntake().repeatedly(), new
		// SetShooterAmp(Math.toRadians(30), 21.27))));
		// operatorController.y().toggleOnTrue(new OnlyFlyWheels(30));

		// //after merge make a parallel command group with turn to speaker
		// controller.x().onTrue(new ManualShootSpeaker(10));
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
	public void teleopInit() {
		// This makes sure that the autonomous stops running when teleop starts running.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		intake.setDefaultCommand(new HomeIntake());
		swerve.setDefaultCommand(new DefaultDrive(true));
		shooter.setDefaultCommand(new HomeShooter());

	}

	@Override
	public void robotPeriodic() {
		// Logger.recordOutput("Example/Mechanism", mechanism);
		CommandScheduler.getInstance().run();

	}
}
