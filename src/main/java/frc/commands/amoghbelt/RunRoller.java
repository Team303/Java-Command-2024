package frc.commands.amoghbelt;

import static frc.robot.Robot.belt;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunRoller extends Command {
    Timer timer = new Timer();

    double start = Integer.MAX_VALUE;

    public RunRoller() {
        addRequirements(belt);
    }

    @Override
    public void initialize() {

        }

    @Override
    public void execute() {
        belt.runRollers();

        // if (!belt.getBeam()) {
        // // start =
        // Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble());
        // }

        // System.out.println("Encoder value: " +
        // Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()));

    }


    @Override
    public void end(boolean interrupted) {
        belt.stopMotors();
    }

}
