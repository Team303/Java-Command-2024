package frc.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class DefaultDrive extends Command {
     boolean fieldOriented;
     boolean togg = false;

     public DefaultDrive(boolean fieldOriented) {
        addRequirements(Robot.swerve);
        this.fieldOriented = fieldOriented;        
     }

     @Override
     public void execute() {
         Translation2d translation;
       
         translation = new Translation2d(
         MathUtil.applyDeadband(-Robot.controller.getLeftY(), 0.15) * Drivetrain.kMaxSpeed *  (1-(Robot.controller.getLeftTriggerAxis()*0.5)),
         MathUtil.applyDeadband(-Robot.controller.getLeftX(), 0.15) * Drivetrain.kMaxSpeed * (1-(Robot.controller.getLeftTriggerAxis()*0.5)));
        
         double rotation = MathUtil.applyDeadband(Robot.controller.getRightX(), 0.15);

         Robot.swerve.drive(translation, rotation * 4 * (1-(Robot.controller.getLeftTriggerAxis()*0.5)), fieldOriented);
     }
}


