package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.DESIRED_LEFT_RPM_ENTRY;
import static frc.subsystems.Shooter.DESIRED_RIGHT_RPM_ENTRY;
import static frc.subsystems.Shooter.INTERPOLATED_DEGREES_ENTRY;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.FieldConstants;


public class DynamicShootSpeaker extends Command {

    double desiredAngle;
    double desiredVelocityLeft;
    double desiredVelocityRight;
    final double FLYWHEEL_SPEED = 21.27;
    Translation2d target;
    double range;


    public DynamicShootSpeaker() {
        addRequirements(shooter);

        desiredVelocityRight = FLYWHEEL_SPEED;

        INTERPOLATED_DEGREES_ENTRY.setDouble(Math.toDegrees(desiredAngle));
        shooter.pivotAngle = desiredAngle;
    }

    @Override
    public void initialize() {
                // find distance to the nearest speaker
        var alliance = DriverStation.getAlliance();

        boolean isBlue = true;
        if (alliance.isPresent()) {
            isBlue = alliance.get() == DriverStation.Alliance.Blue;
        }

        target = isBlue ? FieldConstants.centerSpeakOpenInBlue.getTranslation() : FieldConstants.centerSpeakOpenInRed.getTranslation();

        Pose2d curPose = Robot.swerve.getPose();

        range = Math.hypot(target.getX() - curPose.getX(), target.getY() - curPose.getY());

        desiredAngle = shooter.interpolateAngle(range, desiredVelocityRight > 21.0);
    }

    @Override
    public void execute() {
        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        DESIRED_LEFT_RPM_ENTRY.setDouble(desiredVelocityLeft / (2 * Math.PI * 0.0508) * 60);        
        DESIRED_RIGHT_RPM_ENTRY.setDouble(desiredVelocityRight / (2 * Math.PI * 0.0508) * 60);

        System.out.println("DesiredVelocityLeft: "+((desiredVelocityLeft / (2 * Math.PI * 0.0508))));

        double voltage = shooter.calculateAngleSpeed(desiredAngle);

        System.out.println("Range: " + range);
        System.out.println("Angle: " + desiredAngle);

        System.out.println("Voltage:  " + voltage);
        if (shooter.getAbsoluteShooterAngle() > Math.PI && shooter.getAbsoluteShooterAngle() < Math.toRadians(320) && voltage > 0) {
            shooter.leftAngleMotor.setVoltage(0);
        } else {
            shooter.leftAngleMotor.setVoltage(voltage);
        }

        shooter.leftFlywheelMotor.setControl(shooter.flywheelVoltageLeft.withVelocity((desiredVelocityLeft / (2 * Math.PI * 0.0508))));
        shooter.rightFlywheelMotor.setControl(shooter.flywheelVoltageRight.withVelocity((desiredVelocityRight / (2 * Math.PI * 0.0508))));

    }

    @Override
    public boolean isFinished() {
        return shooter.atSetpoint() 
            && (shooter.leftFlywheelMotor.getVelocity().refresh().getValueAsDouble() > desiredVelocityLeft / (2 * Math.PI * 0.0508))
            && (shooter.rightFlywheelMotor.getVelocity().refresh().getValueAsDouble() > desiredVelocityRight / (2 * Math.PI * 0.0508));
    }

    public void end(boolean interrupted) {
        shooter.leftFlywheelMotor.setVoltage(0);
        shooter.rightFlywheelMotor.setVoltage(0);
        //shooter.leftFlywheelMotor.setVoltage(0);
    }

}
