package frc.commands.shooter;

import static frc.robot.Robot.shooter;

import edu.wpi.first.wpilibj2.command.Command;

public class IntakeFromHuman extends Command {

    public IntakeFromHuman() {
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        
    } 

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        
    }
    
}
