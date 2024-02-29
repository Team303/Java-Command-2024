package frc.commands.amoghbelt;
import static frc.robot.Robot.belt;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotMap;

public class IntakeNote extends Command {

    Timer timer = new Timer();
    double start;

    public IntakeNote() {
        addRequirements(belt);

        BELT_SPEED_ENTRY.setDouble(belt.beltMotor.getVelocity().refresh().getValueAsDouble());
    }

    // @Override
    // public void initialize() {
    // }

    @Override
    public void execute() {
        belt.runBelt();

        // if (!belt.getBeam()) {
            // start = Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble());
        // }

        // System.out.println("Encoder value: " + Math.abs(belt.indexerMotor.getPosition().refresh().getValueAsDouble()));
        
    }

    @Override
    public boolean isFinished() {
        return !belt.getBeam();
    }

    @Override
    public void end(boolean interrupted) {
        // Commands.waitSeconds(0.4);
        belt.stopMotors();
    }

}
