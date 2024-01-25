package frc.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.DriveSubsystem;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DefaultDrive extends Command {
    boolean fieldOriented;
    Translation2d translation;
    double percentPower;

    public DefaultDrive(boolean fieldOriented) {
        addRequirements(Robot.swerve);
        this.fieldOriented = fieldOriented;        
    }

    @Override
    public void execute() {

        percentPower = (1-(Robot.driverController.getLeftTriggerAxis()*0.5));
    
        translation = new Translation2d(
        MathUtil.applyDeadband(-Robot.driverController.getLeftY(), 0.15) * DriveSubsystem.kMaxSpeed * percentPower,
        MathUtil.applyDeadband(-Robot.driverController.getLeftX(), 0.15) * DriveSubsystem.kMaxSpeed * percentPower);
        
        double rotation = MathUtil.applyDeadband(Robot.driverController.getRightX() * percentPower, 0.15) 
            * DriveSubsystem.kMaxAngularSpeed * percentPower;

        Robot.swerve.drive(translation, rotation, fieldOriented);
    }
}


