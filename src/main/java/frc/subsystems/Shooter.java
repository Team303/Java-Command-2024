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
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Shooter extends SubsystemBase {

    /* -------------Kraken Code------------
    private final TalonFX leftAngleMotor;
    private final TalonFX rightAngleMotor;
    private final TalonFX leftFlywheelMotor;
    private final TalonFX rightFlywheelMotor;
    */


    private final CANSparkMax leftAngleMotor;
    private final CANSparkMax rightAngleMotor;
    private final CANSparkMax leftFlywheelMotor;
    private final CANSparkMax rightFlywheelMotor;

    private final SparkLimitSwitch switchReverse;

    private final RelativeEncoder angleEncoder;
    private final RelativeEncoder flywheelEncoder;

    private final BangBangController flywheelController;
    private final SimpleMotorFeedforward flywheelFeedForward;

    private final ArmFeedforward shooterAngleFeedForward;

    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("Turret"); //Shuffleboard tab
    public static final GenericEntry ANGLE_POSITION_ENTRY = SHOOTER_TAB.add("Angle Position", 0.0).withPosition(0, 0).getEntry();
    public static final GenericEntry FLYWHEEL_SPEED_ENTRY = SHOOTER_TAB.add("Flywheel Speed", 0.0).withPosition(1, 0).getEntry();

    public Shooter() {
        /* -------------Kraken Code------------
        leftAngleMotor = new TalonFX(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID);
        rightAngleMotor = new TalonFX(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID);
        leftFlywheelMotor = new TalonFX(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID);
        rightFlywheelMotor = new TalonFX(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID);

        leftAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        rightAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        */

        leftAngleMotor = new CANSparkMax(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID, MotorType.kBrushless);
        rightAngleMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID, MotorType.kBrushless);
        leftFlywheelMotor = new CANSparkMax(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        rightFlywheelMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);

        leftAngleMotor.setIdleMode(IdleMode.kBrake);
        rightAngleMotor.setIdleMode(IdleMode.kBrake);
        leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
        rightFlywheelMotor.setIdleMode(IdleMode.kCoast);

        leftAngleMotor.setInverted(false);
        rightAngleMotor.setInverted(true);
        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);

        angleEncoder = leftAngleMotor.getEncoder();
        flywheelEncoder = leftFlywheelMotor.getEncoder();

        switchReverse = leftAngleMotor.getReverseLimitSwitch(Type.kNormallyOpen);

        flywheelController = new BangBangController();

        flywheelFeedForward = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KV,
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KA);

        shooterAngleFeedForward = new ArmFeedforward(RobotMap.Shooter.ANGLE_FEED_FORWARD_KS, 
                                                     RobotMap.Shooter.ANGLE_FEED_FORWARD_KG, 
                                                     RobotMap.Shooter.ANGLE_FEED_FORWARD_KV, 
                                                     RobotMap.Shooter.ANGLE_FEED_FORWARD_KA);

    }

    public void setSpeed(double speed) {
        leftAngleMotor.set(speed);
        rightAngleMotor.set(speed);
    }

    public void calculateAngleSpeed(double angle) {

    }

    public void setFlywheelSpeed(double speed) {
        leftFlywheelMotor.set(speed);
        rightFlywheelMotor.set(speed);
    }

    public double getShooterAngle() {
        return angleEncoder.getPosition();
    }

    public double getVelocitySpeed() {
        return flywheelEncoder.getVelocity();
    }

	public boolean atHardLimit() {
        return switchReverse.isPressed();
    }

    public void resetEncoders() {
        angleEncoder.setPosition(0);
        flywheelEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
        ANGLE_POSITION_ENTRY.setDouble(angleEncoder.getPosition());
        FLYWHEEL_SPEED_ENTRY.setDouble(flywheelEncoder.getVelocity());
    }
    
}