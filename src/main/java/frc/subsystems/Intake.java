package frc.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;

import frc.robot.RobotMap;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;

public class Intake extends SubsystemBase {

    public static float[] shoulderLimits = { 50, -19.5f }; //CHANGE LATER ARAV >>>>>>>>
	public static float[] elbowLimits = { 170, 30 }; // CHANGE LATER NINJA


    public static final double SHOULDER_START_ANGLE = 4.0; //change por favor ninajo
	public static final double ELBOW_START_ANGLE = 2.0; //change por favor ninajo

    //IDK WHAT THESE ARE CHANGE LATER
	public static final double MAX_VELOCITY = (2 * Math.PI) * 0.35;
	public static final double MAX_VELOCITY_ELBOW = (2 * Math.PI) * 0.4;
	public static final double MAX_VELOCITY_WRIST = MAX_VELOCITY_ELBOW;
	public static final double MAX_ACCELERATION = 64;


    public interface IntakeJoint {
		/**
		 * Sets the speed of the motors
		 * Range is [-1, 1]
		 */
		void setSpeed(double speed);

		/**
		 * Gets the absolute angle of the joint in radians
		 */
		double getJointAngle();

		/**
		 * Gets the motor encoder position in radians
		 */
		double getEncoderPosition();

		/**
		 * Gets the state of the limit switch
		 */
		boolean atHardLimit();

		/**
		 * Computes whether the joint is outside the soft angle constraints
		 */
		default boolean atSoftLimit() {
			return atSoftForwardLimit() || atSoftReverseLimit();
		}

		/**
		 * Returns whether or not the joint is outside its soft forward limit
		 */
		boolean atSoftForwardLimit();

		/**
		 * Returns whether or not the joint is outside its soft reverse limit
		 */
		boolean atSoftReverseLimit();
	}

	public static class ShoulderJoint implements IntakeJoint {

		/* Motors */

		private final TalonFX leftMotor = new TalonFX(RobotMap.Intake.SHOULDER_JOINT_LEFT_ID);
		private final TalonFX rightMotor = new TalonFX(RobotMap.Intake.SHOULDER_JOINT_RIGHT_ID);

		/* Encoders */
		private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(8);

		/* Limits */

		private final DigitalInput shoulderLimit = new DigitalInput(RobotMap.Intake.SHOULDER_JOINT_LEFT_ID);

		/* Controllers */

		private final ProfiledPIDController controller = new ProfiledPIDController(0.6, 0, 0.05,
				new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;
		private MechanismLigament2d real;

		public ShoulderJoint() {
			leftMotor.setNeutralMode(NeutralModeValue.Brake);
			rightMotor.setNeutralMode(NeutralModeValue.Brake);

			leftMotor.setInverted(true);
			rightMotor.setInverted(false);

			// leftMotor.setSmartCurrentLimit(30);
			// rightMotor.setSmartCurrentLimit(30);

            //CONVERSION FACTOR NEEDED
            absoluteEncoder.setDistancePerRotation(2 * Math.PI * (1 / RobotMap.Intake.GEAR_RATIO_SHOULDER));
			controller.setTolerance(Math.toRadians(2));
		}

		@Override
		public void setSpeed(double speed) {
			leftMotor.set(speed);
			rightMotor.set(speed);
		}

		@Override
		public double getJointAngle() {
			return MathUtil.angleModulus(Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition())
					+ Math.toRadians(SHOULDER_START_ANGLE));
		}

		@Override
		public double getEncoderPosition() {
			return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
		}

		@Override
		public boolean atHardLimit() {
			return !shoulderLimit.get();
		}

		@Override
		public boolean atSoftForwardLimit() {
			return this.getJointAngle() > Math.toRadians(shoulderLimits[0]);
		}

		@Override
		public boolean atSoftReverseLimit() {
			return this.getJointAngle() < Math.toRadians(shoulderLimits[1]);
		}
	}
	public static class ElbowJoint implements IntakeJoint {

		private final TalonFX leftMotor = new TalonFX(RobotMap.Intake.ELBOW_JOINT_LEFT_ID);
		private final TalonFX rightMotor = new TalonFX(RobotMap.Intake.ELBOW_JOINT_RIGHT_ID);

		/* Encoders */
		private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(8);

		/* Limits */

		private final DigitalInput shoulderLimit = new DigitalInput(RobotMap.Intake.SHOULDER_JOINT_LEFT_ID);

		/* Controllers */

		private final ProfiledPIDController controller = new ProfiledPIDController(0.6, 0, 0.05,
				new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

		private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
		private MechanismLigament2d simulator;
		private MechanismLigament2d real;


		public ElbowJoint() {
			leftMotor.setNeutralMode(NeutralModeValue.Brake);
			rightMotor.setNeutralMode(NeutralModeValue.Brake);

			leftMotor.setInverted(true);
			rightMotor.setInverted(false);

			// leftMotor.setSmartCurrentLimit(30);
			// rightMotor.setSmartCurrentLimit(30);

            //CONVERSION FACTOR NEEDED
            absoluteEncoder.setDistancePerRotation(2 * Math.PI * (1 / RobotMap.Intake.GEAR_RATIO_SHOULDER));
			controller.setTolerance(Math.toRadians(2));

		}

		@Override
		public void setSpeed(double speed) {
			leftMotor.set(speed);
            rightMotor.set(speed);
		}

		@Override
		public double getJointAngle() {
			return MathUtil.angleModulus(Units.rotationsToRadians(absoluteEncoder.getAbsolutePosition())
					+ Math.toRadians(SHOULDER_START_ANGLE));
		}

		@Override
		public double getEncoderPosition() {
			return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
		}

		@Override
		public boolean atHardLimit() {
			return !shoulderLimit.get();
		}

		@Override
		public boolean atSoftForwardLimit() {
			return this.getJointAngle() > Math.toRadians(shoulderLimits[0]);
		}

		@Override
		public boolean atSoftReverseLimit() {
			return this.getJointAngle() < Math.toRadians(shoulderLimits[1]);
		}
	}

    public final TalonFX rollerMotor;

    public Intake() {
        rollerMotor = new TalonFX(RobotMap.Intake.ROLLER_MOTOR_ID);
    }

    
}
