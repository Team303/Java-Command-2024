package frc.commands.amoghbelt;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Robot.belt;

public class NudgeNote extends Command {

    double start = Integer.MAX_VALUE;

    public NudgeNote(){
        addRequirements(belt);
    }

    @Override
    public void initialize() {
        start = belt.indexerMotor.getPosition().refresh().getValueAsDouble();
    }

    @Override
    public void execute() {
        belt.runBelt(0, 0, 8);
        System.out.println("nudge");
        
        // if (!belt.getBeam()) {
        //     // start = Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble());
        // }

        // System.out.println("Encoder value: " + Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()));
        
    }

    @Override
    public boolean isFinished() {
        return belt.indexerMotor.getPosition().refresh().getValueAsDouble() - start > 7;
    }

    @Override
    public void end(boolean interrupted) {
        belt.stopMotors();
    }

}
