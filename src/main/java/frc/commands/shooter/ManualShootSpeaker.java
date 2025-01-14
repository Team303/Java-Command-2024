package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.DESIRED_LEFT_RPM_ENTRY;
import static frc.subsystems.Shooter.DESIRED_RIGHT_RPM_ENTRY;
import static frc.subsystems.Shooter.INTERPOLATED_DEGREES_ENTRY;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj2.command.Command;

public class ManualShootSpeaker extends Command {

    double desiredAngle;
    double desiredVelocityLeft;
    double desiredVelocityRight;
    final double FLYWHEEL_SPEED = 21.27;

    public ManualShootSpeaker(double range) {
        addRequirements(shooter);

        desiredAngle = shooter.interpolateAngle(range);

        desiredVelocityRight = FLYWHEEL_SPEED;

        INTERPOLATED_DEGREES_ENTRY.setDouble(Math.toDegrees(desiredAngle));
        shooter.pivotAngle = desiredAngle;
        System.out.println("ManualShootSpeaker");
    }

    @Override
    public void execute() {

        System.out.println("Desired Angle: " + desiredAngle * (180 / Math.PI));

        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        DESIRED_LEFT_RPM_ENTRY.setDouble(desiredVelocityLeft / (2 * Math.PI * 0.0508) * 60);
        DESIRED_RIGHT_RPM_ENTRY.setDouble(desiredVelocityRight / (2 * Math.PI * 0.0508) * 60);

        shooter.leftFlywheelMotor
                .setControl(shooter.flywheelVoltageLeft.withVelocity((desiredVelocityLeft / (2 * Math.PI * 0.0508))));
        shooter.rightFlywheelMotor
                .setControl(shooter.flywheelVoltageRight.withVelocity((desiredVelocityRight / (2 * Math.PI * 0.0508))));

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

        shooter.leftAngleMotor.setVoltage(voltage);

    }

    @Override
    public boolean isFinished() {
        return shooter.atSetpoint()
                && (shooter.leftFlywheelMotor.getVelocity().refresh().getValueAsDouble() > desiredVelocityLeft
                        / (2 * Math.PI * 0.0508))
                && (shooter.rightFlywheelMotor.getVelocity().refresh().getValueAsDouble() > desiredVelocityRight
                        / (2 * Math.PI * 0.0508));
    }

    public void end(boolean interrupted) {
        shooter.leftFlywheelMotor.setVoltage(0);
        shooter.rightFlywheelMotor.setVoltage(0);
        // shooter.leftFlywheelMotor.setVoltage(0);
    }

}
