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
import java.util.List;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class Intake extends SubsystemBase {

    public double desiredShoulderAngle;
    public double desiredElbowAngle;

    public static float[] shoulderLimits = { 50, -19.5f }; // CHANGE LATER ARAV >>>>>>>>
    public static float[] elbowLimits = { 170, 30 }; // CHANGE LATER NINJA

    public static final double SHOULDER_START_ANGLE = 4.0; // change por favor ninajo
    public static final double ELBOW_START_ANGLE = 2.0; // change por favor ninajo

	public ShoulderJoint shoulderJoint = new ShoulderJoint();
	public ElbowJoint elbowJoint = new ElbowJoint();

    private double shoulderReachAngle;
	private double elbowReachAngle;

    private final Mechanism2d intakeSimulation;

    public static final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("Intake");

    public static final GenericEntry effectorXEntry = INTAKE_TAB.add("effectorX", 0).withSize(1, 1).withPosition(0, 0).getEntry();
	public static final GenericEntry effectorYEntry = INTAKE_TAB.add("effectorY", 0).withSize(1, 1).withPosition(1, 0).getEntry();

	public static final GenericEntry shoulderAngleEntry = INTAKE_TAB.add("Desired Shoulder Angle", 0).withSize(1, 1).withPosition(0, 1).getEntry();
	public static final GenericEntry elbowAngleEntry = INTAKE_TAB.add("Desired Elbow Angle", 0).withSize(1, 1).withPosition(1, 1).getEntry();

    public static final GenericEntry shoulderEncoderEntry = INTAKE_TAB.add("Shoulder Encoder", 0).withSize(1, 1).withPosition(0, 2).getEntry();
	public static final GenericEntry elbowEncoderEntry = INTAKE_TAB.add("Elbow Encoder", 0).withSize(1, 1).withPosition(1, 2).getEntry();
            
    public static final GenericEntry shoulderEncoderErrorEntry = INTAKE_TAB.add("shoulderEncoderError", 0).withSize(1, 1).withPosition(0, 3).getEntry();
	public static final GenericEntry elbowEncoderErrorEntry = INTAKE_TAB.add("elbowEncoderError", 0).withSize(1, 1).withPosition(1, 3).getEntry();

	public static final GenericEntry shoulderAbsoluteAngleEntry = INTAKE_TAB.add("Shoulder Absolute Angle", 0).withSize(1, 1).withPosition(0, 4).getEntry();
	public static final GenericEntry elbowAbsoluteAngleEntry = INTAKE_TAB.add("Elbow Absolute Angle", 0).withSize(1, 1).withPosition(1, 4).getEntry();

	public static final GenericEntry shoulderSwitchEntry = INTAKE_TAB.add("Shoulder Limit Switch", 0).withSize(1, 1).withPosition(0, 5).getEntry();
	public static final GenericEntry elbowSwitchEntry = INTAKE_TAB.add("Elbow Limit Switch", 0).withSize(1, 1).withPosition(1, 5).getEntry();

    // IDK WHAT THESE ARE CHANGE LATER
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
        private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(7);

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

            // CONVERSION FACTOR NEEDED
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
            return absoluteEncoder.getAbsolutePosition();
            //return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
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

        private final CANSparkMax leftMotor = new CANSparkMax(RobotMap.Intake.ELBOW_JOINT_LEFT_ID, MotorType.kBrushless);
        private final CANSparkMax rightMotor = new CANSparkMax(RobotMap.Intake.ELBOW_JOINT_RIGHT_ID, MotorType.kBrushless);

        /* Encoders */
        private final DutyCycleEncoder absoluteEncoder = new DutyCycleEncoder(8);

        /* Limits */

        private final DigitalInput elbowLimit = new DigitalInput(RobotMap.Intake.ELBOW_JOINT_LEFT_ID);

        /* Controllers */

        private final ProfiledPIDController controller = new ProfiledPIDController(0.6, 0, 0.05,
                new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION));

        private final ArmFeedforward feedForward = new ArmFeedforward(0.01, 0, 0, 0);
        private MechanismLigament2d simulator;
        private MechanismLigament2d real;

        public ElbowJoint() {
            leftMotor.setIdleMode(IdleMode.kBrake);
            rightMotor.setIdleMode(IdleMode.kBrake);

            leftMotor.setInverted(true);
            rightMotor.setInverted(false);

            //leftMotor.setSmartCurrentLimit(30);
            //rightMotor.setSmartCurrentLimit(30);

            // CONVERSION FACTOR NEEDED
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
            return absoluteEncoder.getAbsolutePosition();
            //return (leftMotor.getPosition().getValueAsDouble() + rightMotor.getPosition().getValueAsDouble()) / 2;
        }

        @Override
        public boolean atHardLimit() {
            return !elbowLimit.get();
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
        intakeSimulation = new Mechanism2d(300 / 33.07, 300 / 33.07);

    }
	/**
	 * Reach for a list of raw desired angles
	 * <br>
	 * <br>
	 * Order is [shoulder, elbow]
	 */
	public List<Double> reach(List<Double> desiredRadianAngles) {
		// Pull angles out of list
		desiredShoulderAngle = desiredRadianAngles.get(0);
		desiredElbowAngle = desiredRadianAngles.get(1);

		shoulderReachAngle = desiredShoulderAngle;
		elbowReachAngle = desiredElbowAngle;

		// Compute feedforward
		// TODO: Recompute the angles to fit the inputs wanted by the feedforward
		// controller (relative to horizontal)

		// elbowJoint.controller.setP(((Math.PI / 2 - (desiredShoulderAngle +
		// desiredElbowAngle)) + Math.PI / 2) * 0.5);
		// System.out.println(elbowJoint.controller.getP());

		double shoulderFeedForward = shoulderJoint.feedForward.calculate(Math.PI / 2 - desiredShoulderAngle, 0);
		double elbowFeedForward = elbowJoint.feedForward
				.calculate(Math.PI / 2 - (desiredShoulderAngle + desiredElbowAngle), 0);

		// Compute feedback
		double shoulderFeedback = shoulderJoint.controller.calculate(
				shoulderJoint.getJointAngle(),
				desiredShoulderAngle);
		double elbowFeedback = elbowJoint.controller.calculate(
				elbowJoint.getJointAngle(),
				desiredElbowAngle);

		// Compute joint speeds based on feedback and feedforward while limiting
		// movement based on hard and soft limits

		final double BOUNCE_FORCE = -0.125;

		double shoulderSpeed = shoulderFeedback;
		double elbowSpeed = elbowFeedback;

		boolean forwardShoulderLimit = Math.signum(shoulderSpeed) > 0
				&& shoulderJoint.atSoftForwardLimit();
		boolean reverseShoulderLimit = Math.signum(shoulderSpeed) < 0
				&& shoulderJoint.atSoftReverseLimit();

	

		boolean forwardElbowLimit = Math.signum(elbowSpeed) > 0
				&& elbowJoint.atSoftForwardLimit();
		boolean reverseElbowLimit = Math.signum(elbowSpeed) < 0
				&& elbowJoint.atSoftReverseLimit();

		
	
		move(shoulderSpeed, elbowSpeed);

	
		return desiredRadianAngles;
	}

    public void move(double shoulderSpeed, double elbowSpeed) {
		/* Shoulder */

		boolean forwardShoulderLimit = Math.signum(shoulderSpeed) > 0
				&& shoulderJoint.atSoftForwardLimit();
		boolean reverseShoulderLimit = Math.signum(shoulderSpeed) < 0
				&& (shoulderJoint.atHardLimit() || shoulderJoint.atSoftReverseLimit());

		if (!forwardShoulderLimit && !reverseShoulderLimit) {
			shoulderJoint.setSpeed(shoulderSpeed);
		}

		/* Elbow */

		boolean forwardElbowLimit = Math.signum(elbowSpeed) > 0
				&& (elbowJoint.atHardLimit() || elbowJoint.atSoftForwardLimit());
		boolean reverseElbowLimit = Math.signum(elbowSpeed) < 0
				&& (elbowJoint.atSoftReverseLimit());

		if (!forwardElbowLimit && !reverseElbowLimit) {
			elbowJoint.setSpeed(elbowSpeed);
		}

	}

    @Override
    public void periodic() {
        double shoulderSimAngle = desiredShoulderAngle;
		double elbowSimAngle = desiredElbowAngle;

		if (shoulderSimAngle != this.shoulderReachAngle || elbowSimAngle != this.elbowReachAngle) {
			this.shoulderReachAngle = shoulderSimAngle;
			this.elbowReachAngle = elbowSimAngle;
		}

		shoulderJoint.simulator.setAngle(this.shoulderReachAngle);
		elbowJoint.simulator.setAngle(this.elbowReachAngle);

		shoulderJoint.real.setAngle(-Math.toDegrees(this.shoulderJoint.getJointAngle()) + 90);
		elbowJoint.real.setAngle(-Math.toDegrees(this.elbowJoint.getJointAngle()));


		shoulderEncoderEntry.setDouble(shoulderJoint.getEncoderPosition());
		elbowEncoderEntry.setDouble(elbowJoint.getEncoderPosition());

		shoulderSwitchEntry.setBoolean(shoulderJoint.atHardLimit());
		elbowSwitchEntry.setBoolean(elbowJoint.atHardLimit());

		shoulderAbsoluteAngleEntry.setDouble(Math.toDegrees(shoulderJoint.getJointAngle()));
		elbowAbsoluteAngleEntry.setDouble(Math.toDegrees(elbowJoint.getJointAngle()));

		// shoulderSoftLimitEntry.setBoolean(shoulderJoint.atSoftLimit());
		// elbowSoftLimitEntry.setBoolean(elbowJoint.atSoftLimit());

		// shoulderSoftForwardLimitEntry.setBoolean(shoulderJoint.atSoftForwardLimit());
		// elbowSoftForwardLimitEntry.setBoolean(elbowJoint.atSoftForwardLimit());

		// shoulderSoftReverseLimitEntry.setBoolean(shoulderJoint.atSoftReverseLimit());
		// elbowSoftReverseLimitEntry.setBoolean(elbowJoint.atSoftReverseLimit());

		shoulderEncoderErrorEntry.setDouble(shoulderJoint.getJointAngle() - shoulderJoint.getEncoderPosition());
		elbowEncoderErrorEntry.setDouble(elbowJoint.getJointAngle() - elbowJoint.getEncoderPosition());

		Logger.recordOutput("MyMechanism", this.intakeSimulation);
    }

}
