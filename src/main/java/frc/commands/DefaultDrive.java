package frc.commands;

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

        percentPower = (1-(Robot.controller.getLeftTriggerAxis()*0.5));
    
        translation = new Translation2d(
        MathUtil.applyDeadband(Robot.controller.getLeftX(), 0.25) * DriveSubsystem.kMaxSpeed * percentPower,
        MathUtil.applyDeadband(-Robot.controller.getLeftY(), 0.25) * DriveSubsystem.kMaxSpeed * percentPower);
        
        double rotation = -MathUtil.applyDeadband(Robot.controller.getRightX() * percentPower, 0.2) 
            * DriveSubsystem.kMaxAngularSpeed * percentPower;

        Robot.swerve.drive(translation, rotation, fieldOriented);
    }
}


