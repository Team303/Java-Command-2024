package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;
import static frc.subsystems.Intake.MOTOR_OUTPUT;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotMap;

public class HomeIntake extends Command {

    public HomeIntake() {
        addRequirements(intake);

        DESIRED_PIVOT_ANGLE_ENTRY.setDouble(RobotMap.Intake.HOME_ANGLE);
        intake.pivotAngle = RobotMap.Intake.HOME_ANGLE;
    }

    @Override
    public void initialize() {
        intake.pivotPIDController.setP(3);
    }

    @Override
    public void execute() {
        double voltage = intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE);

        // System.out.println("Voltage: " + voltage);
        System.out.println("Voltage: " + intake.leftPivotMotor.getBusVoltage());
        MOTOR_OUTPUT.setDouble(voltage);

        if (intake.atHomeHardLimit()) {
            intake.rightPivotMotor.setVoltage(-voltage);
        } else if (intake.getAbsolutePivotAngle() > Math.PI / 2 && intake.getAbsolutePivotAngle() < Math.toRadians(320)
                && voltage > 0) {
            // System.out.println("yippee");
            intake.rightPivotMotor.setVoltage(0);
        } else {
            // System.out.println("boooo");
            double percentPower = (14 - (Robot.driverController.getLeftTriggerAxis() * 0.7));
            intake.rightPivotMotor.setVoltage(voltage + (Robot.driverController.getLeftY() * percentPower * -0.7)
                    + Math.abs(Robot.driverController.getRightX() * percentPower * 1.2)); // increase voltage by 1.5 for
                                                                                          // each rpm the robot is
                                                                                          // turning
        }

    }

    // @Override
    // public boolean isFinished() {
    // return (intake.getAbsolutePivotAngle() > Math.PI/2 &&
    // intake.getAbsolutePivotAngle() < Math.PI) ||
    // Robot.intake.pivotPIDController.atSetpoint();
    // }

    // @Override
    // public void end(boolean interrupted) {
    // // Lock the intake
    // intake.leftPivotMotor.setVoltage(0);
    // intake.rightPivotMotor.setVoltage(0);
    // intake.pivotAlan.setPosition(0);

    // }

}
