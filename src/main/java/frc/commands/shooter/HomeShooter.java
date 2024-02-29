package frc.commands.shooter;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Robot.shooter;

import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.Timer;


public class HomeShooter extends Command {

    public HomeShooter() {
        addRequirements(shooter);
    }
    @Override
    public void execute() {


        shooter.leftFlywheelMotor.setVoltage(0);
        shooter.rightFlywheelMotor.setVoltage(0);

        double voltage = shooter.calculateAngleSpeed(Math.toRadians(7));

        System.out.println("Voltage Home: " + voltage);

        // if (Math.abs(shooter.getAbsoluteShooterAngle() - Math.toRadians(7)) < Math.toRadians(4)) {
        //     shooter.leftAngleMotor.setControl(new VelocityVoltage(0));
        //     System.out.println("close enough!!!");
        // } else {
           shooter.leftAngleMotor.setVoltage(voltage);
        // }
    } 

    // @Override
    // public boolean isFinished() {
    //     return shooter.anglePIDController.atSetpoint() || shooter.getAbsoluteShooterAngle() > Math.toRadians(270);
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     shooter.leftAngleMotor.setVoltage(0);
    //     shooter.rightAngleMotor.setVoltage(0);
    // }

}
