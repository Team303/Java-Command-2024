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
    double desiredVelocityRight;
    double desiredVelocityLeft;

    public ManualSetAngle(double height, double range) {
        addRequirements(shooter);
        
        desiredVelocityRight = 17;
        desiredVelocityLeft = 17 * shooter.getFactor();

        desiredAngle = Math.atan((2 * height) / range);
        //Calculate Speed here;
        //desiredVelocity = 0.0;
        //desiredVelocity = Math.sqrt((2*height *9.8*(16*height * height + range * range ))/(8*height));
        // desiredVelocity = Math.sqrt(2 * height * 9.8) / Math.sin(desiredAngle); //new equation
    }


    @Override
    public void execute() {
        // shooter.leftAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        // shooter.rightAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        desiredVelocityLeft = 17 * shooter.getFactor();

        shooter.leftFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeedLeft(desiredVelocityLeft));
        //shooter.rightFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeedRight(desiredVelocityRight));
    } 

    // @Override
    // public boolean isFinished() {
    //     return shooter.getShooterAngle() == desiredAngle && shooter.getVelocitySpeed() == desiredVelocity;
    // }

}