package frc.commands.intake;

import static frc.robot.Robot.intake;
import static frc.robot.Robot.belt;
import static frc.subsystems.Intake.DESIRED_PIVOT_ANGLE_ENTRY;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;
import static frc.subsystems.Intake.MOTOR_OUTPUT;
import static frc.robot.Robot.navX;

public class HomeAlone extends Command{
    
    public HomeAlone() {
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
        // intake.leftPivotMotor.setVoltage(intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE));
        
        intake.pivotPIDController.setP((navX.getRate() / 360)*3);


        double voltage = intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE);

        System.out.println("Voltage: " + voltage);
        MOTOR_OUTPUT.setDouble(voltage);


        if (intake.getAbsolutePivotAngle() < 3 * Math.PI/2 || intake.getAbsolutePivotAngle() > Math.toRadians(345))
            intake.rightPivotMotor.setVoltage(voltage);
        else if (voltage > 0)
            intake.rightPivotMotor.setVoltage(voltage);
        else 
            intake.rightPivotMotor.setVoltage(0);

        MOTOR_OUTPUT.setDouble(intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE));

    }
}
