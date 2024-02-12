package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class HomeIntake extends Command {

    public HomeIntake() {
        addRequirements(intake);

        DESIRED_PIVOT_ANGLE_ENTRY.setDouble(RobotMap.Intake.GROUND_ANGLE);
        intake.pivotAngle = RobotMap.Intake.GROUND_ANGLE;
    }

    @Override
    public void execute() {
        intake.leftPivotMotor.setVoltage(intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE));
        intake.rightPivotMotor.setVoltage(intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE));

        intake.beltMotor.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return intake.atHomeHardLimit() || Math
                .abs(intake.getAbsolutePivotAngle() - RobotMap.Intake.HOME_ANGLE) <= RobotMap.Intake.ANGLE_TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        // Lock the intake
        intake.leftPivotMotor.setVoltage(0);
        intake.rightPivotMotor.setVoltage(0);
    }

}