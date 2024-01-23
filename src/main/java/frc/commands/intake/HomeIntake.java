package frc.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Robot.intake;

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

    }


     
}
