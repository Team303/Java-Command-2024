package frc.commands.drive;

import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

public class DriveWait extends WaitCommand {

	public DriveWait(double seconds) {
		super(seconds);
		addRequirements(Robot.swerve);
	}

}