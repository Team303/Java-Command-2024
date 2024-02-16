package frc.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap.Climber;

public class PullUp extends Command{
    private double speed;
    public PullUp(double speed, double limit) {
        addRequirements(Robot.climber);
        this.speed = speed;
        this.limit = limit;
    }

    @Override
    public void execute() {
        Robot.climber.keepGoing(speed,limit);
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopArms();
    }
}
