package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;
import static frc.subsystems.Intake.MOTOR_OUTPUT;

import edu.wpi.first.math.MathUtil;
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
        // set Kp to 3 to home intake
        intake.pivotPIDController.setP(3);
    }

    @Override
    public void execute() {
        double voltage = intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE);

        // System.out.println("Voltage: " + voltage);
        System.out.println("Voltage: " + intake.rightPivotMotor.getBusVoltage());
        MOTOR_OUTPUT.setDouble(voltage);

        // if the intake is in home position, set voltage to 1

        if (intake.atHomeHardLimit() && voltage > 0) {
            intake.rightPivotMotor.setVoltage(1);
        } else if (intake.getAbsolutePivotAngle() > Math.PI / 2 && intake.getAbsolutePivotAngle() < Math.toRadians(320)
                && voltage > 0) {
            intake.rightPivotMotor.setVoltage(1);
        } else {
            // incerase voltage by 1.5 for each rpm the robot is turning

            // TODO: test this pls
            double accerlationVoltage = MathUtil.applyDeadband(Robot.navX.getRawAccelY() * 0.1, 0.25);
            double omegaVoltage = MathUtil.applyDeadband(Math.abs(Robot.navX.getAngle() * 0.1), 0.25);

            intake.rightPivotMotor.setVoltage(voltage + accerlationVoltage + omegaVoltage);

            System.out.print("accerlationVoltage: " + accerlationVoltage);
            System.out.println("omegaVoltage: " + omegaVoltage);

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
    // intake.rightPivotMotor.setVoltage(0);
    // intake.rightPivotMotor.setVoltage(0);
    // intake.pivotAlan.setPosition(0);

    // }

}
