package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;
import edu.wpi.first.wpilibj2.command.Command;


public class GroundIntake extends Command {
    double desiredAngle; //through bore encoder gives data in degrees

    public GroundIntake(double angle) {
        addRequirements(intake);

        desiredAngle = angle; 
        DESIRED_PIVOT_ANGLE_ENTRY.setDouble(desiredAngle);
        intake.pivotAngle = desiredAngle;
    }

    @Override
    public void execute() {
        intake.leftPivotMotor.setVoltage(intake.calculateAngleSpeed(desiredAngle));
        intake.rightPivotMotor.setVoltage(intake.calculateAngleSpeed(desiredAngle));

        intake.beltMotor.setVoltage(12); // idk maybe -12?

    }
}