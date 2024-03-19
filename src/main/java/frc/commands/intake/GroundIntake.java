
package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;
import static frc.subsystems.Intake.MOTOR_OUTPUT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class GroundIntake extends Command {

    public 
    
    GroundIntake() {
        addRequirements(intake);

        DESIRED_PIVOT_ANGLE_ENTRY.setDouble(RobotMap.Intake.GROUND_ANGLE);
        intake.pivotAngle = RobotMap.Intake.GROUND_ANGLE;
    }

    @Override
    public void initialize() {
        intake.pivotPIDController.setP(3);
    }

    @Override
    public void execute() {
        System.out.println("Ground Intake");
        double voltage = intake.calculateAngleSpeed(RobotMap.Intake.GROUND_ANGLE);
        MOTOR_OUTPUT.setDouble(-voltage);

        // Soft Limit

        if (intake.getAbsolutePivotAngle() > Math.toRadians(265) || intake.getAbsolutePivotAngle() < Math.toRadians(15)) {
            intake.rightPivotMotor.setVoltage(-voltage);        }
        else {
            intake.rightPivotMotor.setVoltage(0);
        }
    }

    @Override
    public boolean isFinished() {
        return (intake.getAbsolutePivotAngle() > Math.toRadians(355)
                && intake.getAbsolutePivotAngle() < Math.toRadians(5)) || intake.getPivotPIDController().atSetpoint();
    }

    // @Override
    // public void end(boolean interrupted) {
    // // Lock the intake
    // intake.rightPivotMotor.setVoltage(0);
    // intake.rightPivotMotor.setVoltage(0);
    // }
}