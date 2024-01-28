package frc.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.subsystems.Shooter;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.BangBangController;
import static frc.robot.Robot.shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class ManualSetAngle extends Command {    
    double desiredAngle;
    double desiredVelocity;

    public ManualSetAngle(double height, double range) {
        addRequirements(shooter);

        desiredVelocity = 30.194977967;
        desiredAngle = Math.atan((2 * height) / range);
        //Calculate Speed here;
        //desiredVelocity = 0.0;
        //desiredVelocity = Math.sqrt((2*height *9.8*(16*height * height + range * range ))/(8*height));
        desiredVelocity = Math.sqrt(2 * height * 9.8) / Math.sin(desiredAngle); //new equation
    }


    @Override
    public void execute() {
        // shooter.leftAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        // shooter.rightAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        shooter.leftFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeed(desiredVelocity));
        shooter.rightFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeed(desiredVelocity));
    } 

    // @Override
    // public boolean isFinished() {
    //     return shooter.getShooterAngle() == desiredAngle && shooter.getVelocitySpeed() == desiredVelocity;
    // }

}