package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; 

public class ShootIndexer extends Command {
    
  Timer timer = new Timer();

    public ShootIndexer() {
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooter.setIndexerSpeed(0.2);
    } 

    @Override
    public boolean isFinished() {
        return  shooter.getBeamBreak() == false || timer.hasElapsed(2.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIndexerSpeed(0);
        timer.stop();
    }

}
