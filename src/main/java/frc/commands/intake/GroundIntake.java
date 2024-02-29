
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
    public void initialize() {
        intake.pivotPIDController.setP(3);
    }

    @Override
    public void execute() {

        double voltage = intake.calculateAngleSpeed(RobotMap.Intake.GROUND_ANGLE);
        System.out.println("Voltage: " + voltage);
        MOTOR_OUTPUT.setDouble(voltage);

        //soft limit 
        

        if (intake.getAbsolutePivotAngle() < 3 * Math.PI/2 || intake.getAbsolutePivotAngle() > Math.toRadians(320))
            intake.rightPivotMotor.setVoltage(voltage);
        else 
            intake.rightPivotMotor.setVoltage(0);
    }

    // @Override
    // public boolean isFinished() {
    //     return (intake.getAbsolutePivotAngle() > Math.toRadians(340) && intake.getAbsolutePivotAngle() < Math.toRadians(350))  || intake.getPivotPIDController().atSetpoint();
    // }

    // @Override
    // public void end(boolean interrupted) {
    //     // Lock the intake
    //     intake.leftPivotMotor.setVoltage(0);
    //     intake.rightPivotMotor.setVoltage(0);

    // }
}