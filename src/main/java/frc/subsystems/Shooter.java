package frc.subsystems;

import static frc.robot.Robot.shooter;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {

    //Flywheel stuff
    public final TalonFX leftFlywheelMotor;
    public final TalonFX rightFlywheelMotor;

    public final BangBangController flywheelBangController;
    public final SimpleMotorFeedforward flywheelFeedForwardLeft;
    public final SimpleMotorFeedforward flywheelFeedForwardRight;

    //Angle stuff
    public final TalonFX leftAngleMotor;
    public final TalonFX rightAngleMotor;

    public final ProfiledPIDController leftShooterAnglePIDController;
    public final ProfiledPIDController rightShooterAnglePIDController;
    public final ArmFeedforward leftShooterAngleFeedForward;
    public final ArmFeedforward rightShooterAngleFeedForward;

    public final DutyCycleEncoder shooterAngleEncoder;

    //Limit Switch Stuff
    public final DigitalInput angleLimitSwitch;

    //Interploation Stuff
    public InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap timeInterpolator = new InterpolatingDoubleTreeMap();

    //We need to interpolate until 12 meters 💀
    public static double[] interpolationAngles = {1.054, 1.013, 0.974, 0.937, 0.902, 0.869, 0.838, 0.809, 0.782, 0.756, //1.0 - 1.9
                                                  0.731, 0.709, 0.687, 0.667, 0.648, 0.629, 0.612, 0.596, 0.581, 0.567, //2.0 - 2.9
                                                  0.553, 0.540, 0.528, 0.516, 0.505, 0.495};                            //3.0 - 3.5

    public static double[] interpolationTimes = new double[interpolationAngles.length];

    //Shuffleboard Stuff
    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("Shooter"); //Shuffleboard tab

        //Flywheel Entries
        public static final GenericEntry DESIRED_LEFT_RPM_ENTRY = SHOOTER_TAB.add("Desired Left RPM", 0.0).withPosition(0, 0).withSize(2, 1).getEntry();
        public static final GenericEntry DESIRED_RIGHT_RPM_ENTRY = SHOOTER_TAB.add("Desired Right RPM", 0.0).withPosition(0, 2).withSize(2, 1).getEntry();
        public static final GenericEntry ACTUAL_LEFT_RPM_ENTRY = SHOOTER_TAB.add("Actual Left RPM", 0.0).withPosition(1, 0).withSize(2, 1).getEntry();
        public static final GenericEntry ACTUAL_RIGHT_RPM_ENTRY = SHOOTER_TAB.add("Actual Right RPM", 0.0).withPosition(1, 2).withSize(2, 1).getEntry();
        public static final GenericEntry RPM_DIFF_FACTOR_ENTRY = SHOOTER_TAB.add("RPM Diff Factor", 0.0).withPosition(2, 0).withSize(1, 1).getEntry();

        //Angle Entries
        public static final GenericEntry INTERPOLATED_DEGREES_ENTRY = SHOOTER_TAB.add("Desired Degrees", 0.0).withPosition(2, 1).withSize(1, 1).getEntry();
        public static final GenericEntry ACTUAL_SHOOTER_ANGLE_ENTRY = SHOOTER_TAB.add("Actual Degrees", 0.0).withPosition(5, 0).getEntry();

        //Limit Entries
        public static final GenericEntry ANGLE_LIMIT_SWITCH_STATUS_ENTRY = SHOOTER_TAB.add("Angle Hard Limit", false).withPosition(0, 0).getEntry();


    //Advantage Scope Sims
    public static Mechanism2d shooterSim;

    //Extras
    public static double diffFactor = 1.0;

    public Shooter() {
        //Flywheel Initalization
        leftFlywheelMotor = new TalonFX(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID);
        rightFlywheelMotor = new TalonFX(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID);

        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);
        
        flywheelBangController = new BangBangController();

        flywheelFeedForwardLeft = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KV,
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KA);

        flywheelFeedForwardRight = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KV,
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KA);



        //Angle Initalization
        TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(4, 2); //Change: Alan's job
        leftAngleMotor = new TalonFX(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID);
        rightAngleMotor = new TalonFX(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID);

        leftAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        rightAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        leftAngleMotor.setInverted(false);
        rightAngleMotor.setInverted(true);

        leftShooterAngleFeedForward = new ArmFeedforward(RobotMap.Shooter.ANGLE_FEED_FORWARD_KS, 
                                                         RobotMap.Shooter.ANGLE_FEED_FORWARD_KG, 
                                                         RobotMap.Shooter.ANGLE_FEED_FORWARD_KV, 
                                                         RobotMap.Shooter.ANGLE_FEED_FORWARD_KA);

        rightShooterAngleFeedForward = new ArmFeedforward(RobotMap.Shooter.ANGLE_FEED_FORWARD_KS, 
                                                          RobotMap.Shooter.ANGLE_FEED_FORWARD_KG, 
                                                          RobotMap.Shooter.ANGLE_FEED_FORWARD_KV, 
                                                          RobotMap.Shooter.ANGLE_FEED_FORWARD_KA);

        leftShooterAnglePIDController = new ProfiledPIDController(RobotMap.Shooter.ANGLE_PID_CONTROLLER_P, 
                                                                  RobotMap.Shooter.ANGLE_PID_CONTROLLER_I,
                                                                  RobotMap.Shooter.ANGLE_PID_CONTROLLER_D, 
                                                                  pidConstraints);
        
        rightShooterAnglePIDController = new ProfiledPIDController(RobotMap.Shooter.ANGLE_PID_CONTROLLER_P, 
                                                                   RobotMap.Shooter.ANGLE_PID_CONTROLLER_I,
                                                                   RobotMap.Shooter.ANGLE_PID_CONTROLLER_D, 
                                                                   pidConstraints);

        leftShooterAnglePIDController.enableContinuousInput(-180, 180);
        rightShooterAnglePIDController.enableContinuousInput(-180, 180);

        shooterAngleEncoder = new DutyCycleEncoder(RobotMap.Shooter.ANGLE_ENCODER_ID);
        //shooterAngleEncoder.setDistancePerRotation(diffFactor); -- Use Gear Ratio Prob


        //Limit Switch Initalization
        angleLimitSwitch = new DigitalInput(RobotMap.Shooter.ANGLE_HARD_STOP_SWITCH_ID);

        //Interpolation Initalization
        initalizeInterpolationTable();

        //Current Limit Initalization
        CurrentLimitsConfigs clc = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
        
        leftFlywheelMotor.getConfigurator().apply(clc);
        rightFlywheelMotor.getConfigurator().apply(clc);
        leftAngleMotor.getConfigurator().apply(clc);
        rightAngleMotor.getConfigurator().apply(clc);

        resetFlywheelEncoders();
    }


    //Flywheel Functions
    public double calculateFlywheelSpeedLeft(double speed) {
        final double bangOutput = flywheelBangController.calculate(leftFlywheelMotor.getVelocity().refresh().getValueAsDouble(), (speed / (2 * Math.PI * 0.0508)));
        final double flywheelFeedForwardOutput = flywheelFeedForwardLeft.calculate(speed);

        
        DESIRED_LEFT_RPM_ENTRY.setDouble(speed / (2 * Math.PI * 0.0508) * 60);
        return (0.6 * bangOutput) + (flywheelFeedForwardOutput);
    }

    public double calculateFlywheelSpeedRight(double speed) {
        final double bangOutput = flywheelBangController.calculate(rightFlywheelMotor.getVelocity().refresh().getValueAsDouble(), (speed / (2 * Math.PI * 0.0508)));
        final double flywheelFeedForwardOutput = flywheelFeedForwardRight.calculate(speed);

        DESIRED_RIGHT_RPM_ENTRY.setDouble(speed / (2 * Math.PI * 0.0508) * 60);
        return (0.6 * bangOutput) + (flywheelFeedForwardOutput);

    }

    public double getVelocitySpeedLeft() {
        return leftFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }

    public double getVelocitySpeedRight() {
        return rightFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }


    //Angle Functions
    public double calculateAngleSpeedLeft(double angle) {
         final double angleOutput = leftShooterAnglePIDController.calculate(getAbsoluteShooterAngle(), angle);
         final double angleFeedforward = leftShooterAngleFeedForward.calculate(angle, leftShooterAnglePIDController.getSetpoint().velocity);
         return angleOutput + angleFeedforward;
    }


    public double calculateAngleSpeedRight(double angle) {
         final double angleOutput = rightShooterAnglePIDController.calculate(getAbsoluteShooterAngle(), angle);
         final double angleFeedforward = rightShooterAngleFeedForward.calculate(angle, rightShooterAnglePIDController.getSetpoint().velocity);
         return angleOutput + angleFeedforward;
    }

    public double getAbsoluteShooterAngle() {
        return shooterAngleEncoder.getAbsolutePosition();
    }

	public boolean atHardLimit() {
         return !angleLimitSwitch.get();
    }

    public void resetFlywheelEncoders() {
        leftFlywheelMotor.setPosition(0);
        rightFlywheelMotor.setPosition(0);
    }

    //Interpolation Functions
    public void initalizeInterpolationTable() {
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

    //Factor Functions
    public double getFactor() {
        return diffFactor;
    }

    public void setFactor(double factor) {
        diffFactor = factor;
    }

    @Override
    public void periodic() {
        //Flywheel Entries
        ACTUAL_LEFT_RPM_ENTRY.setDouble(getVelocitySpeedLeft() * 60);
        ACTUAL_RIGHT_RPM_ENTRY.setDouble(getVelocitySpeedRight() * 60);
        RPM_DIFF_FACTOR_ENTRY.setDouble(diffFactor);

        //Angle Entries
        ACTUAL_SHOOTER_ANGLE_ENTRY.setDouble(Math.toDegrees(getAbsoluteShooterAngle()));

        //Limit Entries
        ANGLE_LIMIT_SWITCH_STATUS_ENTRY.setBoolean(atHardLimit());


        // Logger.recordOutput("Shooter Simulation", shooterSim);
    }
    
}