// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
// import org.littletonrobotics.junction.Logger;
// import org.littletonrobotics.junction.networktables.NT4Publisher;
// import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
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
import frc.commands.drive.DefaultDrive;
import frc.commands.drive.DriveWait;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.subsystems.Drivetrain;
import frc.subsystems.Intake;

public class Robot extends Logged {
  public static final CommandXboxController controller = new CommandXboxController(0);
  public static final AHRS navX = new AHRS(); 
  public static final Drivetrain swerve = new Drivetrain();
  public static final Intake intake = new Intake();


  @Override
  public void robotInit() {
    configureButtonBindings();

    Autonomous.init();
		AutonomousProgram.addAutosToShuffleboard();
  }

  private void configureButtonBindings() {
    controller.y().onTrue(new InstantCommand(swerve::resetOdometry));
  }

  	/* Currently running auto routine */

	private Command autonomousCommand;

  @Override
	public void autonomousInit() {
		navX.reset();
		swerve.resetOdometry();
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
    swerve.setDefaultCommand(new DefaultDrive(true));
	}

  @Override
  public void teleopPeriodic() { 

	  CommandScheduler.getInstance().run();
		
  }
}
