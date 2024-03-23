package frc.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.subsystems.DriveSubsystem;

public class GoForward extends Command {
    Translation2d translation;
    double percentPower;
    double dist;
    double start;

    boolean isAlliance = true;
 
    public GoForward(double dist) {
        addRequirements(Robot.swerve);
        this.dist=dist;
    }

    @Override
    public void initialize() {
        start = Math.abs(Robot.swerve.frontRight.getDrivePosition());
    }

    @Override
    public void execute() {
        percentPower = 1;
        Translation2d translation = new Translation2d(
                2.9,
                0);

        double rotation = 0;


        Robot.swerve.drive(translation, rotation, false);
    }

    public boolean isFinished(){
        return Math.abs(Robot.swerve.frontRight.getDrivePosition()) - start > dist;
    }
}