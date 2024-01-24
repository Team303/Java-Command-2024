package frc.commands;

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
        Robot.climber.extendArms();
    }

    @Override
    public boolean isFinished() {
        return Robot.climber.isArmOneAtLimit() && Robot.climber.isArmOneAtLimit();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.climber.stopArms();
    }
}
