package frc.subsystems;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Intake extends SubsystemBase {

	public final CANSparkMax leftPivotMotor;
	public final CANSparkMax rightPivotMotor;

	public final ProfiledPIDController pivotPIDController;
	public final ArmFeedforward pivotFeedForward;

	public final TalonFX beltMotor;

	public final DutyCycleEncoder pivotEncoder;

	public final DigitalInput homeLimit;
	public final DigitalInput groundLimit;

	public static final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("Intake"); // Shuffleboard tab
	public static final GenericEntry DESIRED_PIVOT_ANGLE_ENTRY = INTAKE_TAB.add("Desired Degrees", 0.0)
			.withPosition(0, 0).getEntry();
	public static final GenericEntry ACTUAL_PIVOT_ANGLE_ENTRY = INTAKE_TAB.add("Actual Degrees", 0.0).withPosition(1, 0)
			.getEntry();
	public static final GenericEntry BELT_SPEED_ENTRY = INTAKE_TAB.add("Belt Speed", 0.0).getEntry();



	public double pivotAngle = 0.0;

	public Intake() {
		beltMotor = new TalonFX(RobotMap.Intake.BELT_MOTOR_ID);
		beltMotor.setNeutralMode(NeutralModeValue.Coast);

		CurrentLimitsConfigs clc40 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
		beltMotor.getConfigurator().apply(clc40);


		TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(4, 2); // Change: Alan's job
		leftPivotMotor = new CANSparkMax(RobotMap.Intake.LEFT_PIVOT_MOTOR_ID, MotorType.kBrushless);
		rightPivotMotor = new CANSparkMax(RobotMap.Intake.RIGHT_PIVOT_MOTOR_ID, MotorType.kBrushless);

		leftPivotMotor.setIdleMode(IdleMode.kBrake);
		rightPivotMotor.setIdleMode(IdleMode.kBrake);

		pivotFeedForward = new ArmFeedforward(RobotMap.Intake.PIVOT_FEED_FORWARD_KS,
				RobotMap.Intake.PIVOT_FEED_FORWARD_KG,
				RobotMap.Intake.PIVOT_FEED_FORWARD_KV,
				RobotMap.Intake.PIVOT_FEED_FORWARD_KA);

		pivotPIDController = new ProfiledPIDController(RobotMap.Intake.PIVOT_PID_CONTROLLER_P,
				RobotMap.Intake.PIVOT_PID_CONTROLLER_I,
				RobotMap.Intake.PIVOT_PID_CONTROLLER_D,
				pidConstraints);


		leftPivotMotor.follow(rightPivotMotor, true);

		leftPivotMotor.setSmartCurrentLimit(40);
		rightPivotMotor.setSmartCurrentLimit(40);

		pivotEncoder = new DutyCycleEncoder(RobotMap.Intake.PIVOT_ENCODER_ID);
		homeLimit = new DigitalInput(RobotMap.Intake.HOME_LIMIT_SWITCH_ID);
		groundLimit = new DigitalInput(RobotMap.Intake.GROUND_LIMIT_SWITCH_ID);

	}

	// pivot functions
	public double calculateAngleSpeed(double angle) {

		final double pivotOutput = pivotPIDController.calculate(getAbsolutePivotAngle(), angle);
		final double pivotFeedforward = pivotFeedForward.calculate(angle, pivotPIDController.getSetpoint().velocity);

		TrapezoidProfile.Constraints constrain = new TrapezoidProfile.Constraints(
			3.14/2, 
			pivotFeedForward.maxAchievableAcceleration(12, angle, leftPivotMotor.getEncoder().getVelocity())); // Change: Alan's job


		pivotPIDController.setConstraints(constrain);
		return pivotOutput + pivotFeedforward;
	}

	public double getAbsolutePivotAngle() {
		return pivotEncoder.getAbsolutePosition();
	}

	public boolean atHomeHardLimit() {
		return !homeLimit.get();
	}

	public boolean atGroundHardLimit() {
		return !groundLimit.get();
	}

	@Override
	public void periodic() {
		ACTUAL_PIVOT_ANGLE_ENTRY.setDouble(getAbsolutePivotAngle());
	}

}










