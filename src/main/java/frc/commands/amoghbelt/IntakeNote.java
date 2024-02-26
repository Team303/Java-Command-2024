package frc.commands.amoghbelt;
import static frc.robot.Robot.belt;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class IntakeNote extends Command {

    public IntakeNote() {
        addRequirements(belt);

        BELT_SPEED_ENTRY.setDouble(belt.beltMotor.getVelocity().refresh().getValueAsDouble());
    }

    @Override
    public void execute() {
        belt.runBelt();
    }

    @Override
    public boolean isFinished() {
        return belt.getBeam();
    }

    @Override
    public void end(boolean interrupted) {
        belt.stopMotors();
    }

}
