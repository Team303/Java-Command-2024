package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.belt;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import static frc.subsystems.Intake.MOTOR_OUTPUT;

public class GroundIntake extends Command {

    public GroundIntake() {
        addRequirements(intake);

        DESIRED_PIVOT_ANGLE_ENTRY.setDouble(RobotMap.Intake.GROUND_ANGLE);
        intake.pivotAngle = RobotMap.Intake.GROUND_ANGLE;
    }

    @Override
    public void execute() {
        // intake.leftPivotMotor.setVoltage(intake.calculateAngleSpeed(RobotMap.Intake.GROUND_ANGLE));
        // intake.rightPivotMotor.setVoltage(intake.calculateAngleSpeed(RobotMap.Intake.GROUND_ANGLE));
        intake.rightPivotMotor.setVoltage(4);
        MOTOR_OUTPUT.setDouble(intake.calculateAngleSpeed(RobotMap.Intake.GROUND_ANGLE));
    }

    @Override
    public boolean isFinished() {
        return intake.getAbsolutePivotAngle() < Math.toRadians(-10) || intake.getPivotPIDController().atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        // Lock the intake
        intake.leftPivotMotor.setVoltage(0);
        intake.rightPivotMotor.setVoltage(0);


    }
}