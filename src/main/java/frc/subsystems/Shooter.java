package frc.subsystems;

import java.util.HashMap;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import frc.robot.RobotMap;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

    // -------------Kraken Code------------
    // public final TalonFX leftAngleMotor;
    // public final TalonFX rightAngleMotor;
    // public final TalonFX leftFlywheelMotor;
    // public final TalonFX rightFlywheelMotor;
    

    //spark max code
    // public final CANSparkMax leftAngleMotor;
    // public final CANSparkMax rightAngleMotor;
    public final CANSparkMax leftFlywheelMotor;
    public final CANSparkMax rightFlywheelMotor;

    // public final CANSparkMax leftIndexerMotor;
    // public final CANSparkMax rightIndexerMotor;

    //public final LimitSwitchNormal switchReverse;

    // public final DutyCycleEncoder angleEncoder_dutyCycle;

    // public final RelativeEncoder indexerEncoder;
    public final RelativeEncoder flyWheelEncoderLeft;
    public final RelativeEncoder flyWheelEncoderRight;

    public final BangBangController flywheelControllerLeft;
    public final BangBangController flywheelControllerRight;
    public final SimpleMotorFeedforward flywheelFeedForwardLeft;
    public final SimpleMotorFeedforward flywheelFeedForwardRight;


    public final ProfiledPIDController shooterAnglePIDController;
    public final ArmFeedforward shooterAngleFeedForward;

    // public final DigitalInput beamBreak;
    // public final DigitalInput angleLimitSwitch;

    public static Mechanism2d shooterSim;

    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("Shooter"); //Shuffleboard tab
    public static final GenericEntry ANGLE_POSITION_ENTRY = SHOOTER_TAB.add("Angle Position", 0.0).withPosition(0, 0).getEntry();
    public static final GenericEntry INTERPOLATED_ANGLE = SHOOTER_TAB.add("Interpolated Angle", 0.0).withPosition(1, 1).getEntry();
    public static final GenericEntry FLYWHEEL_SPEED_ENTRY_LEFT = SHOOTER_TAB.add("Flywheel Speed Left", 0.0).withPosition(1, 0).getEntry();
    public static final GenericEntry FLYWHEEL_SPEED_ENTRY_RIGHT = SHOOTER_TAB.add("Flywheel Speed Right", 0.0).withPosition(2, 0).getEntry();
    public static final GenericEntry DESIRED_SPEED = SHOOTER_TAB.add("Desired Speed", 0.0).withPosition(1, 0).getEntry();


    public static final GenericEntry INDEXER_POSITION_ENTRY = SHOOTER_TAB.add("Indexer Position", 0.0).withPosition(0, 1).getEntry();

    public static final GenericEntry DIFF_FACTOR = SHOOTER_TAB.add("Diff Factor", 0.0).withPosition(3, 0).getEntry();
    public static double diffFactor = 1.0;

    //public static HashMap<Integer, Double> interpolateAngleTable = new HashMap<Integer, Double>();
    //public static DecimalFormat dFormatter = new DecimalFormat("0.0");
    public InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
    public static double[] interpolationAngles = {1.054, 1.013, 0.974, 0.937, 0.902, 0.869, 0.838, 0.809, 0.782, 0.756, //1.0 - 1.9
                                                  0.731, 0.709, 0.687, 0.667, 0.648, 0.629, 0.612, 0.596, 0.581, 0.567, //2.0 - 2.9
                                                  0.553, 0.540, 0.528, 0.516, 0.505, 0.495};                            //3.0 - 3.5
    public InterpolatingDoubleTreeMap timeInterpolator = new InterpolatingDoubleTreeMap();
    public static double[] interpolationTimes = {};

    public Shooter() {
        //addInterpolationTableValues();
        //-------------Kraken Code------------
        // leftAngleMotor = new TalonFX(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID);
        // rightAngleMotor = new TalonFX(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID);
        // leftFlywheelMotor = new TalonFX(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID);
        // rightFlywheelMotor = new TalonFX(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID);

        // leftAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        // rightAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        // leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        // rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        
        //spark mxa codd


        // leftAngleMotor = new CANSparkMax(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID, MotorType.kBrushless);
        // rightAngleMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID, MotorType.kBrushless);
        leftFlywheelMotor = new CANSparkMax(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        rightFlywheelMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID, MotorType.kBrushless);
        // leftIndexerMotor = new CANSparkMax(RobotMap.Shooter.LEFT_INDEXER_MOTOR_ID, MotorType.kBrushless);
        // rightIndexerMotor = new CANSparkMax(RobotMap.Shooter.RIGHT_INDEXER_MOTOR_ID, MotorType.kBrushless);

        // leftAngleMotor.setIdleMode(IdleMode.kBrake);
        // rightAngleMotor.setIdleMode(IdleMode.kBrake);
        leftFlywheelMotor.setIdleMode(IdleMode.kCoast);
        rightFlywheelMotor.setIdleMode(IdleMode.kCoast);
        // leftIndexerMotor.setIdleMode(IdleMode.kBrake);
        // rightIndexerMotor.setIdleMode(IdleMode.kBrake);

        // leftAngleMotor.setInverted(false);
        // rightAngleMotor.setInverted(true);
        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);

        leftFlywheelMotor.setSmartCurrentLimit(40);
        rightFlywheelMotor.setSmartCurrentLimit(40);


        // leftIndexerMotor.setInverted(true);
        // rightIndexerMotor.setInverted(false);

        // angleEncoder_dutyCycle = new DutyCycleEncoder(8);
        // indexerEncoder = leftIndexerMotor.getEncoder();

        flyWheelEncoderLeft = leftFlywheelMotor.getEncoder();
        flyWheelEncoderRight = rightFlywheelMotor.getEncoder();
        // flyWheelEncoder.setVelocityConversionFactor(2 * Math.PI * 0.0508);

        flywheelControllerRight = new BangBangController();
        flywheelControllerLeft = new BangBangController();

        flywheelFeedForwardLeft = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KV,
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KA);

        flywheelFeedForwardRight = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
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

        // beamBreak = new DigitalInput(RobotMap.Shooter.BEAM_BREAK_ID);
        // angleLimitSwitch = new DigitalInput(0);
        for(int i=0;i<interpolationAngles.length;i++){
            angleInterpolator.put((double)i*0.1+1.0,interpolationAngles[i]);
        }
        for(int i=0;i<interpolationTimes.length;i++){
            timeInterpolator.put((double)i*0.1+1.0,interpolationTimes[i]);
        }

    }

    public double interpolateAngle(double range) {
       return angleInterpolator.get(range);
    }

    public double interpolateTime(double range) {
        return timeInterpolator.get(range);
    }

    public double getFactor() {
        return diffFactor;
    }

    public void setFactor(double factor) {
        diffFactor = factor;
    }



    // public double calculateAngleSpeed(double angle) {
    //     final double angleOutput = shooterAnglePIDController.calculate(angleEncoder_dutyCycle.getAbsolutePosition(), angle);
    //     final double angleFeedforward = shooterAngleFeedForward.calculate(angle, shooterAnglePIDController.getSetpoint().velocity);
    //     return angleOutput + angleFeedforward;
    // }

    public double calculateFlywheelSpeedRight(double speed) {
        final double bangOutput = flywheelControllerRight.calculate(flyWheelEncoderRight.getVelocity(), (speed / (2 * Math.PI * 0.0508)) * 60);
        final double flywheelFeedForwardOutput = flywheelFeedForwardRight.calculate(speed);

        return (bangOutput) + (flywheelFeedForwardOutput * 0.9);

    }

    public double calculateFlywheelSpeedLeft(double speed) {
        final double bangOutput = flywheelControllerLeft.calculate(flyWheelEncoderLeft.getVelocity(), (speed / (2 * Math.PI * 0.0508)) * 60);
        final double flywheelFeedForwardOutput = flywheelFeedForwardLeft.calculate(speed);

        
        DESIRED_SPEED.setDouble(speed / (2 * Math.PI * 0.0508) * 60);
        return (bangOutput) + (flywheelFeedForwardOutput * 0.9);
    }

    // public void setAngleSpeed(double speed) {
    //     leftAngleMotor.set(speed);
    //     rightAngleMotor.set(speed);
    // }

    public void setFlywheelSpeed(double speed) {
        leftFlywheelMotor.set(speed);
        rightFlywheelMotor.set(speed);
    }

    // public void setIndexerSpeed(double speed) {
    //     leftIndexerMotor.set(speed);
    //     rightIndexerMotor.set(speed);
    // }

    // public double getShooterAngle() {
    //     return angleEncoder_dutyCycle.getAbsolutePosition();
    // }

    public double getVelocitySpeed() {
        return flyWheelEncoderRight.getVelocity();
    }

	// public boolean atHardLimit() {
    //     return !angleLimitSwitch.get();
    // }

    // public boolean getBeamBreak() {
    //     return !beamBreak.get();
    // }

    public void resetEncoders() {
        flyWheelEncoderRight.setPosition(0);
    }

    @Override
    public void periodic() {
        // ANGLE_POSITION_ENTRY.setDouble(angleEncoder_dutyCycle.getAbsolutePosition());
        FLYWHEEL_SPEED_ENTRY_RIGHT.setDouble(flyWheelEncoderRight.getVelocity());
        FLYWHEEL_SPEED_ENTRY_LEFT.setDouble(flyWheelEncoderLeft.getVelocity());


        DIFF_FACTOR.setDouble(diffFactor);
        Logger.recordOutput("Speed Left", flyWheelEncoderLeft.getVelocity());
        Logger.recordOutput("Speed Right", flyWheelEncoderLeft.getVelocity());
        //FLYWHEEL_RPM.setDouble(leftFlywheelMotor.get)
        // INDEXER_POSITION_ENTRY.setDouble(indexerEncoder.getPosition());
    }
    
}