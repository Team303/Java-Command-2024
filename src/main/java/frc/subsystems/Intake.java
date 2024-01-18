package frc.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.RobotMap;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

    /*
    public final TalonFX leftBottomSegmentMotor;
    public final TalonFX rightBottomSegmentMotor;
    public final TalonFX intakeRollerMotor;
    */

    public final CANSparkMax leftBottomSegmentMotor;
    public final CANSparkMax rightBottomSegmentMotor;
    public final CANSparkMax leftTopSegmentMotor;
    public final CANSparkMax rightTopSegmentMotor;
    public final CANSparkMax intakeRollerMotor;

    public Intake() {
        leftBottomSegmentMotor = new CANSparkMax(RobotMap.Intake.LEFT_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        rightBottomSegmentMotor = new CANSparkMax(RobotMap.Intake.RIGHT_BOTTOM_MOTOR_ID, MotorType.kBrushless);
        leftTopSegmentMotor = new CANSparkMax(RobotMap.Intake.LEFT_TOP_MOTOR_ID, MotorType.kBrushless);
        rightTopSegmentMotor = new CANSparkMax(RobotMap.Intake.RIGHT_TOP_MOTOR_ID, MotorType.kBrushless);
        intakeRollerMotor = new CANSparkMax(RobotMap.Intake.INTAKE_ROLLER_MOTOR_ID, MotorType.kBrushless);

        leftBottomSegmentMotor.setIdleMode(IdleMode.kBrake);
        rightBottomSegmentMotor.setIdleMode(IdleMode.kBrake);
        leftTopSegmentMotor.setIdleMode(IdleMode.kBrake);
        rightTopSegmentMotor.setIdleMode(IdleMode.kBrake);
        intakeRollerMotor.setIdleMode(IdleMode.kBrake);


        leftBottomSegmentMotor.setInverted(false);
        rightBottomSegmentMotor.setInverted(true);
        leftTopSegmentMotor.setInverted(false);
        rightTopSegmentMotor.setInverted(true);
        
    }
    
}
