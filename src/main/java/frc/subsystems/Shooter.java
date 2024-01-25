package frc.subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkLimitSwitch.Type;

import frc.robot.Robot;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;

public class Shooter extends SubsystemBase {

    // -------------Kraken Code------------
    public final TalonFX leftAngleMotor;
    public final TalonFX rightAngleMotor;
    public final TalonFX leftFlywheelMotor;
    public final TalonFX rightFlywheelMotor;
    

    //spark max code
    // public final CANSparkMax leftAngleMotor;
    // public final CANSparkMax rightAngleMotor;
    // public final CANSparkMax leftFlywheelMotor;
    // public final CANSparkMax rightFlywheelMotor;

    public final CANSparkMax leftIndexerMotor;
    public final CANSparkMax rightIndexerMotor;

    //public final LimitSwitchNormal switchReverse;

    public final DutyCycleEncoder angleEncoder_dutyCycle;

    public final RelativeEncoder indexerEncoder;

    public final BangBangController flywheelController;
    public final SimpleMotorFeedforward flywheelFeedForward;

    public final ProfiledPIDController shooterAnglePIDController;
    public final ArmFeedforward shooterAngleFeedForward;

    public final DigitalInput beamBreak;
    public final DigitalInput angleLimitSwitch;

    public static Mechanism2d shooterSim;

    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("Shooter"); //Shuffleboard tab
    public static final GenericEntry ANGLE_POSITION_ENTRY = SHOOTER_TAB.add("Angle Position", 0.0).withPosition(0, 0).getEntry();
    public static final GenericEntry FLYWHEEL_SPEED_ENTRY = SHOOTER_TAB.add("Flywheel Speed", 0.0).withPosition(1, 0).getEntry();
    public static final GenericEntry INDEXER_POSITION_ENTRY = SHOOTER_TAB.add("Indexer Position", 0.0).withPosition(2, 0).getEntry();

    public Shooter() {
        //-------------Kraken Code------------
        leftAngleMotor = new TalonFX(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID);
        rightAngleMotor = new TalonFX(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID);
        leftFlywheelMotor = new TalonFX(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID);
        rightFlywheelMotor = new TalonFX(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID);

        leftAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        rightAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        
        //spark mxa code
        // leftAngleMotor = new CANSparkMax(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID, MotorType.kBrushless);
        // rightAngleMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID, MotorType.kBrushless);
        // leftFlywheelMotor = new CANSparkMax(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        // rightFlywheelMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        leftIndexerMotor = new CANSparkMax(RobotMap.Shooter.LEFT_INDEXER_MOTOR_ID, MotorType.kBrushless);
        rightIndexerMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_INDEXER_MOTOR_ID, MotorType.kBrushless);

        // leftAngleMotor.setIdleMode(IdleMode.kBrake);
        // rightAngleMotor.setIdleMode(IdleMode.kBrake);
        // leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
        // rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
        leftIndexerMotor.setIdleMode(IdleMode.kBrake);
        rightIndexerMotor.setIdleMode(IdleMode.kBrake);

        leftAngleMotor.setInverted(false);
        rightAngleMotor.setInverted(true);
        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);
        leftIndexerMotor.setInverted(true);
        rightIndexerMotor.setInverted(false);

        angleEncoder_dutyCycle = new DutyCycleEncoder(8);
        indexerEncoder = leftIndexerMotor.getEncoder();

        flywheelController = new BangBangController();

        flywheelFeedForward = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KV,
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KA);

        shooterAngleFeedForward = new ArmFeedforward(RobotMap.Shooter.ANGLE_FEED_FORWARD_KS, 
                                                     RobotMap.Shooter.ANGLE_FEED_FORWARD_KG, 
                                                     RobotMap.Shooter.ANGLE_FEED_FORWARD_KV, 
                                                     RobotMap.Shooter.ANGLE_FEED_FORWARD_KA);

        TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(4, 2); //Change: Alan's job

        shooterAnglePIDController = new ProfiledPIDController(RobotMap.Shooter.ANGLE_PID_CONTROLLER_P, 
                                                      RobotMap.Shooter.ANGLE_PID_CONTROLLER_I,
                                                      RobotMap.Shooter.ANGLE_PID_CONTROLLER_D, 
                                                      pidConstraints);

        beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK_ID);
        angleLimitSwitch = new DigitalInput(0);

    }



    public double calculateAngleSpeed(double angle) {
        final double angleOutput = shooterAnglePIDController.calculate(angleEncoder_dutyCycle.getAbsolutePosition(), angle);
        final double angleFeedforward = shooterAngleFeedForward.calculate(angle, shooterAnglePIDController.getSetpoint().velocity);
        return angleOutput + angleFeedforward;
    }

    public double calculateFlywheelSpeed(double speed) {
        final double bangOutput = flywheelController.calculate(leftFlywheelMotor.getDutyCycle().getValueAsDouble(), speed);
        final double flywheelFeedForwardOutput = flywheelFeedForward.calculate(speed);
        return (bangOutput * 12.0) + (flywheelFeedForwardOutput * 0.9);
    }

    public void setAngleSpeed(double speed) {
        leftAngleMotor.set(speed);
        rightAngleMotor.set(speed);
    }

    public void setFlywheelSpeed(double speed) {
        leftFlywheelMotor.set(speed);
        rightFlywheelMotor.set(speed);
    }

    public void setIndexerSpeed(double speed) {
        leftIndexerMotor.set(speed);
        rightIndexerMotor.set(speed);
    }

    public double getShooterAngle() {
        return angleEncoder_dutyCycle.getAbsolutePosition();
    }

    public double getVelocitySpeed() {
        return leftFlywheelMotor.getVelocity().getValueAsDouble();
    }

	public boolean atHardLimit() {
        return !angleLimitSwitch.get();
    }

    public boolean getBeamBreak() {
        return !beamBreak.get();
    }

    public void resetEncoders() {
        leftFlywheelMotor.setPosition(0);
    }

    @Override
    public void periodic() {
        ANGLE_POSITION_ENTRY.setDouble(angleEncoder_dutyCycle.getAbsolutePosition());
        FLYWHEEL_SPEED_ENTRY.setDouble(leftFlywheelMotor.getVelocity().getValueAsDouble());
        INDEXER_POSITION_ENTRY.setDouble(indexerEncoder.getPosition());
    }
    
}