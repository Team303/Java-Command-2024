package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.DESIRED_LEFT_RPM_ENTRY;
import static frc.subsystems.Shooter.DESIRED_RIGHT_RPM_ENTRY;
import static frc.subsystems.Shooter.INTERPOLATED_DEGREES_ENTRY;

import com.ctre.phoenix6.controls.VelocityVoltage;

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
        System.out.println("At Setpoint: " + shooter.atSetpoint());
        System.out.println("SetShooterAmp");
        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        DESIRED_LEFT_RPM_ENTRY.setDouble(desiredVelocityLeft / (2 * Math.PI * 0.0508) * 60);
        DESIRED_RIGHT_RPM_ENTRY.setDouble(desiredVelocityRight / (2 * Math.PI * 0.0508) * 60);

        System.out.println("DesiredVelocityLeft: " + ((desiredVelocityLeft / (2 * Math.PI * 0.0508))));

        double voltage = shooter.calculateAngleSpeed(desiredAngle);

        System.out.println("Voltage:  " + voltage);

        if (shooter.getAbsoluteShooterAngle() > Math.PI && shooter.getAbsoluteShooterAngle() < Math.toRadians(320)
                && voltage > 0) {
            shooter.leftAngleMotor.setVoltage(0);
        } else if (Math.abs(shooter.getAbsoluteShooterAngle() - desiredAngle) < Math.toRadians(3)) {
            shooter.leftAngleMotor.setControl(new VelocityVoltage(0));
        } else {
            shooter.leftAngleMotor.setVoltage(voltage);
        }

        shooter.leftFlywheelMotor.setControl(shooter.flywheelVoltageLeft.withVelocity((desiredVelocityLeft / (2 * Math.PI * 0.0508))));
        shooter.rightFlywheelMotor.setControl(shooter.flywheelVoltageRight.withVelocity((desiredVelocityRight / (2 * Math.PI * 0.0508))));
    }

    @Override
    public boolean isFinished() {
        return shooter.atSetpoint()  && (shooter.rightFlywheelMotor.getVelocity().refresh().getValueAsDouble() > desiredVelocityRight
                        / (2 * Math.PI * 0.0508));
    }

    public void end(boolean interrupted) {
        shooter.leftFlywheelMotor.setVoltage(0);
        shooter.rightFlywheelMotor.setVoltage(0);
        // shooter.leftFlywheelMotor.setVoltage(0);
    }

}
