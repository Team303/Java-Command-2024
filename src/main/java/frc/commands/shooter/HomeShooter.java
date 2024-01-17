package frc.commands.shooter;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Robot.shooter;
import edu.wpi.first.wpilibj.Timer;


public class HomeShooter extends Command {

    Timer timer = new Timer();

    public HomeShooter() {
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooter.setAngleSpeed(-0.2);
    } 

    @Override
    public boolean isFinished() {
        return shooter.atHardLimit() || timer.hasElapsed(3.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setAngleSpeed(0);
        shooter.resetEncoders();
        timer.stop();
    }

}