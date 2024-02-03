package frc.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.List;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.geometry.Translation3d;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class Intake extends SubsystemBase {

	public static TalonFX pivotMotor;
	public static TalonFX beltMotor;
	
	public static DutyCycleEncoder pivotEncoder;

	public Intake() {
		pivotMotor = new TalonFX(RobotMap.Intake.PIVOT_MOTOR_ID);
		beltMotor = new TalonFX(RobotMap.Intake.BELT_MOTOR_ID);

		pivotMotor.setNeutralMode(NeutralModeValue.Brake);
		beltMotor.setNeutralMode(NeutralModeValue.Brake);

		pivotEncoder = new DutyCycleEncoder(RobotMap.Intake.PIVOT_ENCODER_ID);
	}

	@Override
	public void periodic() {
	}

}
