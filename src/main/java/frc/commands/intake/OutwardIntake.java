
package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;
import static frc.subsystems.Intake.MOTOR_OUTPUT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class OutwardIntake extends Command {

    public OutwardIntake() {
        addRequirements(intake);

        DESIRED_PIVOT_ANGLE_ENTRY.setDouble(80);
        intake.pivotAngle = RobotMap.Intake.GROUND_ANGLE;
    }

    @Override
    public void initialize() {
        intake.pivotPIDController.setP(3);
    }

    @Override
    public void execute() {

        double voltage = intake.calculateAngleSpeed(Math.toRadians(80));
        System.out.println("Voltage: " + voltage);
        MOTOR_OUTPUT.setDouble(voltage);

        // soft limit

        if (intake.getAbsolutePivotAngle() > Math.PI / 2 && intake.getAbsolutePivotAngle() < Math.toRadians(320)
                && voltage > 0)
            intake.rightPivotMotor.setVoltage(0);
        else
            intake.rightPivotMotor.setVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(intake.getAbsolutePivotAngle() - Math.toRadians(80)) < 1;
    }

    // @Override
    // public void end(boolean interrupted) {
    // // Lock the intake
    // intake.leftPivotMotor.setVoltage(0);
    // intake.rightPivotMotor.setVoltage(0);

    // }
}