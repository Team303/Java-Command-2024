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

    public ManualSetAngle(double angle) {
        addRequirements(shooter);
        desiredAngle = angle;
        //Calculate Speed here;
        desiredVelocity = 0.0;
    }


    @Override
    public void execute() {
        shooter.leftAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        shooter.rightAngleMotor.setVoltage(shooter.calculateAngleSpeed(desiredAngle));
        shooter.leftFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeed(desiredVelocity));
        shooter.rightFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeed(desiredVelocity));
    } 

    @Override
    public boolean isFinished() {
        return shooter.getShooterAngle() == desiredAngle && shooter.getVelocitySpeed() == desiredVelocity;
    }

    @Override
    public void end(boolean interrupted) {
         shooter.setAngleSpeed(0);
    }

}