package frc.commands.intake;

import static frc.robot.Robot.intake;
import java.util.Arrays;
import java.util.List;
import edu.wpi.first.wpilibj2.command.Command;


public class MoveToPosition extends Command {

    private List<Double> desiredAngles;

    public MoveToPosition(double shoulderAngle, double elbowAngle, double clawAngle) {
        addRequirements(intake);
        desiredAngles = Arrays.asList(shoulderAngle, elbowAngle, clawAngle);
    }

    @Override
    public void execute() {
        intake.reach(desiredAngles);
    }
}