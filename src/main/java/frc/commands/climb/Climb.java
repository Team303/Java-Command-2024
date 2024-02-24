package frc.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class Climb extends Command {

    public Climb() {
        addRequirements(Robot.climber);
    }

    @Override
    public void initialize() {
        // any setup code can go here
    }

    @Override
    public void execute() {
        Robot.climber.extendArms(0.5);
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
