package frc.commands.shooter;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Robot.shooter;
import edu.wpi.first.wpilibj.Timer;


public class HomeShooter extends Command {

    public HomeShooter() {
        addRequirements(shooter);
    }
    @Override
    public void execute() {


        shooter.leftFlywheelMotor.setVoltage(0);
        shooter.rightAngleMotor.setVoltage(0);

        double voltage = shooter.calculateAngleSpeed(0);

        System.out.println("Voltage Home: " + shooter.calculateAngleSpeed(0));

        shooter.leftAngleMotor.setVoltage(shooter.calculateAngleSpeed(0));
    } 

    @Override
    public boolean isFinished() {
        return shooter.anglePIDController.atSetpoint() || (shooter.getAbsoluteShooterAngle() < Math.toRadians(4) || shooter.getAbsoluteShooterAngle() > Math.toRadians(270));
    }

    @Override
    public void end(boolean interrupted) {
        shooter.leftAngleMotor.setVoltage(0);
        shooter.rightAngleMotor.setVoltage(0);
    }

}
