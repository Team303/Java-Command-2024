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
import edu.wpi.first.wpilibj2.command.Command;

public class DynamicSetAngle extends Command {    

    public DynamicSetAngle(double range, double height, double hood_kP) {
        addRequirements(Robot.shooter);
    }

}