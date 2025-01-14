package frc.commands.amoghbelt;

import static frc.robot.Robot.belt;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ReverseIntakeBBC extends Command {

    Timer timer = new Timer();
    double start;
    int count = 0;

    public ReverseIntakeBBC() {
        addRequirements(belt);
        count = 0;
        BELT_SPEED_ENTRY.setDouble(belt.beltMotor.getVelocity().refresh().getValueAsDouble());
    }

  

    @Override
    public void execute() {
        belt.runIndexerInReverse();

        if (!belt.getBeam()) {
            count++;
        }

     

    }

    @Override
    public boolean isFinished() {
        return !belt.getBeam();
    }

    @Override
    public void end(boolean interrupted) {
        // Commands.waitSeconds(0.4);
        belt.stopMotors();
        count=0;
    }

}
