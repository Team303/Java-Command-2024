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

public class HardcodedSetAngle extends Command {    

    public HardcodedSetAngle(double angle) {
        addRequirements(shooter);
    }


    @Override
    public void execute() {
        shooter.setSpeed(-0.2);
    } 

    @Override
    public boolean isFinished() {
        return 0;
    }

    @Override
    public void end(boolean interrupted) {
    }

}