package frc.commands.drive;

import javax.xml.crypto.dsig.Transform;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class DriveToNote extends Command {

    PIDController driveController;
    PIDController turnController;

    public DriveToNote() {  
        addRequirements(Robot.swerve);
        addRequirements(Robot.detector);

        driveController = new PIDController(0.1, 0, 0.1);
        turnController = new PIDController(0.1, 0, 0.1);

        turnController.enableContinuousInput(-Math.PI, Math.PI);
        turnController.setTolerance(0.2);

    }

    @Override
    public boolean isFinished() {
        return turnController.atSetpoint() && driveController.atSetpoint();
    }

    @Override 
    public void execute() {
        
        Transform3d notePos = Robot.detector.getPosition(); 

        double angle = Math.atan(notePos.getZ()/notePos.getX());
        double distance = Math.hypot(notePos.getX(), notePos.getZ());

        Robot.swerve.drive(new Translation2d(driveController.calculate(distance, 0), 0), turnController.calculate(Robot.navX.getRotation2d().getRadians(), angle), false);
    }

}
