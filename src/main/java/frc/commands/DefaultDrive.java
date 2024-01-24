package frc.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.subsystems.Drivetrain;
import frc.robot.Robot;
import frc.robot.RobotMap;
import frc.subsystems.Drivetrain;

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
        MathUtil.applyDeadband(-Robot.driverController.getLeftY(), 0.15) * Drivetrain.kMaxSpeed * percentPower,
        MathUtil.applyDeadband(-Robot.driverController.getLeftX(), 0.15) * Drivetrain.kMaxSpeed * percentPower);
        
        double rotation = MathUtil.applyDeadband(Robot.driverController.getRightX() * percentPower, 0.15) 
            * Drivetrain.kMaxAngularSpeed * percentPower;

        Robot.swerve.drive(translation, rotation, fieldOriented);
    }
}


