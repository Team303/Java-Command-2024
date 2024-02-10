package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.Climber;

public class Retract extends Command{
    private double speed;
    public Retract(double speed) {
        addRequirements(Robot.climber);
        this.speed = speed; 
    }

    @Override
    public void execute() {
        Robot.climber.retractArms(speed);
    }

    @Override
    public boolean isFinished() {
        return Robot.climber.isArmAtLimit(1) && Robot.climber.isArmAtLimit(2);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopArms();
    }
}
