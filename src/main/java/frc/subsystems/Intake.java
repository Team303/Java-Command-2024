package frc.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

    public final CANSparkMax leftArmMotor;
    public final CANSparkMax rightArmMotor;
    public final CANSparkMax intakeRollerMotor;

    /*
    public final TalonFX intakeRollerMotor;
    */

    public Intake() {
        leftArmMotor = new CANSparkMax(RobotMap.Intake.LEFT_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        rightArmMotor = new CANSparkMax(RobotMap.Intake.RIGHT_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        intakeRollerMotor = new CANSparkMax(RobotMap.Intake.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);

        leftArmMotor.setIdleMode(IdleMode.kBrake);
        rightArmMotor.setIdleMode(IdleMode.kBrake);
        intakeRollerMotor.setIdleMode(IdleMode.kBrake);

        leftArmMotor.setInverted(false);
        rightArmMotor.setInverted(true);
    }
    
}
