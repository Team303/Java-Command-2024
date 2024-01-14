package frc.commands;

import frc.robot.Robot;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class DriveWait extends WaitCommand {

	public DriveWait(double seconds) {
		super(seconds);
		addRequirements(Robot.swerve);
	}

}
