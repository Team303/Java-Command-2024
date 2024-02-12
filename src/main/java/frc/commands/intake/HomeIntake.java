package frc.commands.intake;

import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Robot.intake;
import edu.wpi.first.wpilibj.Timer;


public class HomeIntake extends Command {

    Timer timer = new Timer();

    public HomeIntake() {
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
       //  intake.setAngleSpeed(-0.2);
    } 

    // @Override
    // public boolean isFinished() {
    //     return shooter.atHardLimit() || timer.hasElapsed(3.0);
    // }

    @Override
    public void end(boolean interrupted) {
        // shooter.setAngleSpeed(0);
        
        timer.stop();
    }

}