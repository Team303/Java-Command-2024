package frc.commands.shooter;

import static frc.robot.Robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import static frc.subsystems.Shooter.LERPING_ANGLE_ENTRY;

import com.ctre.phoenix6.controls.VelocityVoltage;

import static frc.subsystems.Shooter.DESIRED_LEFT_RPM_ENTRY;
import static frc.subsystems.Shooter.DESIRED_RIGHT_RPM_ENTRY;
import static frc.subsystems.Shooter.INTERPOLATED_DEGREES_ENTRY;

public class LerpingShooterTest extends Command {

    double desiredAngleRad;
    double desiredVelocityLeft;
    double desiredVelocityRight;

    public LerpingShooterTest(double velocityMetersPerSecond) {
        addRequirements(shooter);

        desiredVelocityRight = velocityMetersPerSecond;


        desiredAngleRad = Math.toRadians(LERPING_ANGLE_ENTRY.getDouble(1));

        desiredAngleRad = Math.min(desiredAngleRad, Math.toRadians(55));
        INTERPOLATED_DEGREES_ENTRY.setDouble(Math.toDegrees(desiredAngleRad));
        shooter.pivotAngle = desiredAngleRad;


    }

    
    @Override
    public void execute() {
        //System.out.println("At Setpoint: " + shooter.atSetpoint());
        //System.out.println("SetShooterAmp");
        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        DESIRED_LEFT_RPM_ENTRY.setDouble(desiredVelocityLeft / (2 * Math.PI * 0.0508) * 60);
        DESIRED_RIGHT_RPM_ENTRY.setDouble(desiredVelocityRight / (2 * Math.PI * 0.0508) * 60);

        //System.out.println("DesiredVelocityLeft: " + ((desiredVelocityLeft / (2 * Math.PI * 0.0508))));

        double voltage = shooter.calculateAngleSpeed(desiredAngleRad);

        //System.out.println("Voltage:  " + voltage);

        if (shooter.getAbsoluteShooterAngle() > Math.PI && shooter.getAbsoluteShooterAngle() < Math.toRadians(320)
                && voltage > 0) {
            shooter.leftAngleMotor.setVoltage(0);
        } else if (Math.abs(shooter.getAbsoluteShooterAngle() - desiredAngleRad) < Math.toRadians(3)) {
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
