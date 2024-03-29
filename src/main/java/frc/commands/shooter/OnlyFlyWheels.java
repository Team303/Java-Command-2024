package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import static frc.subsystems.Shooter.DESIRED_LEFT_RPM_ENTRY;
import static frc.subsystems.Shooter.DESIRED_RIGHT_RPM_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;

public class OnlyFlyWheels extends Command {

    double desiredVelocityLeft;
    double desiredVelocityRight;

    public OnlyFlyWheels(double velocityMetersPerSecond) {
        addRequirements(shooter);

        desiredVelocityRight = velocityMetersPerSecond;
    }

    @Override
    public void execute() {
        desiredVelocityLeft = desiredVelocityRight * shooter.getFactor();

        DESIRED_LEFT_RPM_ENTRY.setDouble(desiredVelocityLeft / (2 * Math.PI * 0.0508) * 60);
        DESIRED_RIGHT_RPM_ENTRY.setDouble(desiredVelocityRight / (2 * Math.PI * 0.0508) * 60);

        System.out.println("DesiredVelocityLeft: " + ((desiredVelocityLeft / (2 * Math.PI * 0.0508))));

        shooter.leftFlywheelMotor
                .setControl(shooter.flywheelVoltageLeft.withVelocity((desiredVelocityLeft / (2 * Math.PI * 0.0508))));
        shooter.rightFlywheelMotor
                .setControl(shooter.flywheelVoltageRight.withVelocity((desiredVelocityRight / (2 * Math.PI * 0.0508))));

    }

    @Override
    public void end(boolean interreupted) {
        shooter.leftAngleMotor.setVoltage(0);
        shooter.leftFlywheelMotor.setVoltage(0);
        shooter.rightFlywheelMotor.setVoltage(0);
    }

}
