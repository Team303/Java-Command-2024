package frc.commands.intake;

import static frc.robot.Robot.intake;

import edu.wpi.first.wpilibj2.command.Command;

public class MoveVolts extends Command {
    double volts;

    public MoveVolts(double volts) {
        addRequirements(intake);
        this.volts = volts;
    }

    @Override
    public void execute() {
        intake.rightPivotMotor.setVoltage(volts);

        // intake.rightPivotMotor.setVoltage(volts);
    }
}
