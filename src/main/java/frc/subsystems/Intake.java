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

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

public class Intake extends SubsystemBase {

	public final TalonFX leftPivotMotor;
	

	public final TalonFX rightPivotMotor;

	public final ProfiledPIDController pivotPIDController;
    public final ArmFeedforward pivotFeedForward;
	
	public final CANSparkMax beltMotor;
	
	public final DutyCycleEncoder pivotEncoder; 

	public static final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("Intake"); //Shuffleboard tab
	public static final GenericEntry DESIRED_PIVOT_ANGLE_ENTRY = INTAKE_TAB.add("Desired Degrees", 0.0).withPosition(0, 0).getEntry();
	public static final GenericEntry ACTUAL_PIVOT_ANGLE_ENTRY = INTAKE_TAB.add("Actual Degrees", 0.0).withPosition(1, 0).getEntry();

	public static Mechanism2d intakeSim;
    public static MechanismLigament2d simulatorRoot;
    public static MechanismLigament2d realRoot;

	public double pivotAngle = 0.0;

	public Intake() {
		TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(4, 2); //Change: Alan's job
		leftPivotMotor = new TalonFX(RobotMap.Intake.LEFT_PIVOT_MOTOR_ID);
		rightPivotMotor = new TalonFX(RobotMap.Intake.RIGHT_PIVOT_MOTOR_ID);

		leftPivotMotor.setNeutralMode(NeutralModeValue.Brake);
		rightPivotMotor.setNeutralMode(NeutralModeValue.Brake);

		pivotFeedForward = new ArmFeedforward(RobotMap.Intake.PIVOT_FEED_FORWARD_KS, 
											  RobotMap.Intake.PIVOT_FEED_FORWARD_KG, 
											  RobotMap.Intake.PIVOT_FEED_FORWARD_KV, 
											  RobotMap.Intake.PIVOT_FEED_FORWARD_KA);

        pivotPIDController = new ProfiledPIDController(RobotMap.Intake.PIVOT_PID_CONTROLLER_P, 
                                                       RobotMap.Intake.PIVOT_PID_CONTROLLER_I,
                                                       RobotMap.Intake.PIVOT_PID_CONTROLLER_D, 
													   pidConstraints);
        

        pivotPIDController.enableContinuousInput(-180, 180);
		rightPivotMotor.setControl(new Follower(leftPivotMotor.getDeviceID(), true));

		CurrentLimitsConfigs clc40 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
		leftPivotMotor.getConfigurator().apply(clc40);
		rightPivotMotor.getConfigurator().apply(clc40);



		beltMotor = new CANSparkMax(RobotMap.Intake.BELT_MOTOR_ID, MotorType.kBrushless);
		beltMotor.setIdleMode(IdleMode.kBrake);
		beltMoto
		pivotEncoder = new DutyCycleEncoder(RobotMap.Intake.PIVOT_ENCODER_ID);

		intakeSim = new Mechanism2d(300 / 33.07, 300 / 33.07);
        MechanismRoot2d intakeRoot = intakeSim.getRoot("Intake", 
                                                         (RobotMap.Intake.SIMULATION_OFFSET + 150) / RobotMap.Intake.SIMULATION_SCALE,
                                                         (RobotMap.Intake.SIMULATION_OFFSET + 150) / RobotMap.Intake.SIMULATION_SCALE);
        
        simulatorRoot = intakeRoot.append(
                new MechanismLigament2d(
                        "Arm",
                        (double) (RobotMap.Intake.INTAKE_SIM_LENGTH) / RobotMap.Intake.SIMULATION_SCALE,
                        0,
                        5.0,
                        new Color8Bit(255, 0, 0)));

        realRoot = intakeRoot.append(
                new MechanismLigament2d(
                        "Arm Real",
                        (double) (RobotMap.Intake.INTAKE_SIM_LENGTH) / RobotMap.Intake.SIMULATION_SCALE,
                        0,
                        5.0,
                        new Color8Bit(255, 255, 0)));
	
	}
	//pivot functions
	public double calculateAngleSpeed(double angle) {
         final double pivotOutput = pivotPIDController.calculate(getAbsoluteIntakeAngle(), angle);
         final double pivotFeedforward = pivotFeedForward.calculate(angle, pivotPIDController.getSetpoint().velocity);
         return pivotOutput + pivotFeedforward;
    } 
	
	public double getAbsoluteIntakeAngle() {
        return pivotEncoder.getAbsolutePosition();
    }

    public double getAbsolutePivotAngle() {
        return pivotEncoder.getAbsolutePosition();
    }

	public boolean atHardLimit() {
		return false; //idk if were using a limit switch but ill keep this here anyways
	   //return !angleLimitSwitch.get();
	}

	//belt functions

	public double getBeltSpeed() {
        return beltMotor.get();
    }

	//getters and setters for motors
	public static TalonFX getLeftPivotMotor() {
		return leftPivotMotor;
	}
	public static void setLeftPivotMotor(TalonFX leftPivotMotor) {
		Intake.leftPivotMotor = leftPivotMotor;
	}
	public static TalonFX getRightPivotMotor() {
		return rightPivotMotor;
	}
	public static void setRightPivotMotor(TalonFX rightPivotMotor) {
		Intake.rightPivotMotor = rightPivotMotor;
	}
	public static CANSparkMax getBeltMotor() {
		return beltMotor;
	}
	public static void setBeltMotor(CANSparkMax beltMotor) {
		Intake.beltMotor = beltMotor;
	}

	@Override
	public void periodic() {
		ACTUAL_PIVOT_ANGLE_ENTRY.setDouble(getAbsolutePivotAngle());

		realRoot.setAngle(getAbsolutePivotAngle());
		simulatorRoot.setAngle(pivotAngle);

		SmartDashboard.putData("IntakeSim", intakeSim);
		Logger.recordOutput("IntakeMechanism", intakeSim);
	}

}