package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.DESIRED_LEFT_RPM_ENTRY;
import static frc.subsystems.Shooter.DESIRED_RIGHT_RPM_ENTRY;
import static frc.subsystems.Shooter.INTERPOLATED_DEGREES_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;

public class SetShooterAmp extends Command {

    double desiredAngle;
    double desiredVelocityLeft;
    double desiredVelocityRight;

    public SetShooterAmp(double angleRad, double velocityMetersPerSecond) {
        addRequirements(shooter);

        desiredVelocityRight = velocityMetersPerSecond;

        desiredAngle = angleRad;
        INTERPOLATED_DEGREES_ENTRY.setDouble(Math.toDegrees(desiredAngle));
        shooter.pivotAngle = desiredAngle;
    }

    @Override
    public void execute() {
        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        DESIRED_LEFT_RPM_ENTRY.setDouble(desiredVelocityLeft / (2 * Math.PI * 0.0508) * 60);        
        DESIRED_RIGHT_RPM_ENTRY.setDouble(desiredVelocityRight / (2 * Math.PI * 0.0508) * 60);

        System.out.println("DesiredVeloicityLeft: "+((desiredVelocityLeft / (2 * Math.PI * 0.0508))));

        shooter.leftFlywheelMotor.setControl(shooter.flywheelVoltageLeft.withVelocity((desiredVelocityLeft / (2 * Math.PI * 0.0508))));
        shooter.rightFlywheelMotor.setControl(shooter.flywheelVoltageRight.withVelocity((desiredVelocityRight / (2 * Math.PI * 0.0508))));

        double voltage = shooter.calculateAngleSpeed(desiredAngle);

        System.out.println("Voltage:  " + voltage);

        shooter.leftAngleMotor.setVoltage(voltage);

    }

    @Override
    public boolean isFinished() {
        return shooter.atSetpoint();
    }

    @Override
    public void end(boolean interreupted) {
        shooter.leftAngleMotor.setVoltage(0);
    }

}