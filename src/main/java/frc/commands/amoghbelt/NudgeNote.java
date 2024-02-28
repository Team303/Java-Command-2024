package frc.commands.amoghbelt;
import edu.wpi.first.wpilibj2.command.Command;
import static frc.robot.Robot.belt;

public class NudgeNote extends Command {

    public NudgeNote(){
        addRequirements(belt);
    }

    @Override
    public void execute() {
        belt.runBelt();
        System.out.println("nudge");
        
        // if (!belt.getBeam()) {
        //     // start = Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble());
        // }

        // System.out.println("Encoder value: " + Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()));
        
    }

}
