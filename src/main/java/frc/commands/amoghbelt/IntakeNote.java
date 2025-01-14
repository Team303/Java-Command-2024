package frc.commands.amoghbelt;

import static frc.robot.Robot.belt;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeNote extends Command {

    Timer timer = new Timer();
    double start;
    int count = 0;

    public IntakeNote() {
        addRequirements(belt);
        count = 0;
        BELT_SPEED_ENTRY.setDouble(belt.beltMotor.getVelocity().refresh().getValueAsDouble());
    }

    // @Override
    // public void initialize() {
    // }

    @Override
    public void execute() {
        belt.runBelt();

        if (!belt.getBeam()) {
            count++;
        }

        // if (!belt.getBeam()) {
        // start =
        // Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble());
        // }

        // System.out.println("Encoder value: " +
        // Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()));

    }

    @Override
    public boolean isFinished() {
        System.out.println("Count: "+count);
        return count > 2;
    }

    @Override
    public void end(boolean interrupted) {
        // Commands.waitSeconds(0.4);
        belt.stopMotors();
        count=0;
    }

}
