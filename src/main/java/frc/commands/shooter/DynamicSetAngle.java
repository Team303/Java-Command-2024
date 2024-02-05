package frc.commands.shooter;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;
import frc.subsystems.Shooter;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.RobotMap;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicSetAngle extends Command {    

    double desiredAngle = 0.0;
    double desiredVelocity = 0.0;

    public DynamicSetAngle(double range, double height) {
        addRequirements(Robot.shooter);

        desiredAngle = Math.atan(2*height/range);
        //desiredVelocity = Math.sqrt((2*height *9.8*(16*height * height + range * range ))/(8*height));
        desiredVelocity = Math.sqrt(2 * height * 9.8) / Math.sin(desiredAngle); //new equation
    }

}