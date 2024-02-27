package frc.commands.amoghbelt;
import static frc.robot.Robot.belt;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class IntakeNote extends Command {

    Timer timer = new Timer();
    double start;

    public IntakeNote() {
        addRequirements(belt);

        BELT_SPEED_ENTRY.setDouble(belt.beltMotor.getVelocity().refresh().getValueAsDouble());
    }

    @Override
    public void initialize() {
        start = Integer.MAX_VALUE;
        timer.reset();
    }

    @Override
    public void execute() {
        belt.runBelt();

        if (!belt.getBeam()) {
            start = Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble());
        }

        System.out.println("Encoder value: " + Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()));
        
    }

    @Override
    public boolean isFinished() {
        return Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()) - start > 0.001;
    }

    @Override
    public void end(boolean interrupted) {
        belt.stopMotors();
    }

}
