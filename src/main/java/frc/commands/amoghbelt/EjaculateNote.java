package frc.commands.amoghbelt;

import static frc.robot.Robot.belt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class EjaculateNote extends Command {
    Timer timer = new Timer();

    double start = Integer.MAX_VALUE;

    public EjaculateNote() {
        addRequirements(belt);
    }

    @Override
    public void initialize() {
        start = belt.indexerMotor.getPosition().refresh().getValueAsDouble();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        belt.runBeltInReverse();
        System.out.println("Time: " + timer.get());

        // if (!belt.getBeam()) {
        // // start =
        // Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble());
        // }

        // System.out.println("Encoder value: " +
        // Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()));

    }

    @Override
    public boolean isFinished() {
        return timer.get() > 2.0;
    }

    @Override
    public void end(boolean interrupted) {
        belt.stopMotors();
    }

}
