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
        intake.pivotPIDController.setP(12);
    }

    @Override
    public void execute() {
        
        //increase P will make it oscillate more probably
        // intake.pivotPIDController.setP((navX.getRate() / 360)*3);


        double voltage = intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE);

        System.out.println("Voltage: " + voltage);
        MOTOR_OUTPUT.setDouble(voltage);

        

        if (intake.getAbsolutePivotAngle() < 3 * Math.PI/2 || intake.getAbsolutePivotAngle() > Math.toRadians(320))
            intake.rightPivotMotor.setVoltage(voltage); // + (navX.getRate() / 360 * 1.5)); // increase voltage by 1.5 for each rpm the robot is turning
        else 
            intake.rightPivotMotor.setVoltage(0);

        MOTOR_OUTPUT.setDouble(intake.calculateAngleSpeed(RobotMap.Intake.HOME_ANGLE));

    }
}