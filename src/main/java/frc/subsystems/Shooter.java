package frc.subsystems;

import static frc.robot.Robot.shooter;

import java.util.HashMap;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Robot;
import frc.robot.RobotMap;
import com.ctre.phoenix6.controls.Follower;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
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
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class Shooter extends SubsystemBase {

    //Flywheel stuff
    public final TalonFX leftFlywheelMotor;
    public final TalonFX rightFlywheelMotor;

    public final BangBangController flywheelLeftBangController;
    public final BangBangController flywheelRightBangController;

    public final SimpleMotorFeedforward flywheelFeedForwardLeft;
    public final SimpleMotorFeedforward flywheelFeedForwardRight;


    public final VelocityVoltage flywheelVoltageLeft = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    public final VelocityVoltage flywheelVoltageRight = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    //public final MotionMagicVelocityDutyCycle flywheelVoltageOutputLeft = new MotionMagicVelocityDutyCycle(0, 0, true, 0, 0, true, false, false);
    //public final MotionMagicVelocityDutyCycle flywheelVoltageOutputRight = new MotionMagicVelocityDutyCycle(0, 0, true, 0, 0, true, false, false);

    //Angle stuff
    public final TalonFX leftAngleMotor;
    public final TalonFX rightAngleMotor;

    public final ProfiledPIDController anglePIDController;
    public final ArmFeedforward angleFeedForward;

    public final DutyCycleEncoder angleEncoder;

    //Limit Switch Stuff
    //public final DigitalInput angleLimitSwitch;

    //Interploation Stuff
    public InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap timeInterpolator = new InterpolatingDoubleTreeMap();

    //We need to interpolate until 12 meters 💀
    public static double[] interpolationAngles = {1.060, 0.961, 0.876, 0.802, 0.738, 0.683, 0.635, 0.594, 0.559, 0.527, 0.500,  //1.00 --> 3.750
                                                  0.476, 0.454, 0.435, 0.419, 0.404, 0.390, 0.378, 0.367, 0.357, 0.349, 0.341,  //3.00 --> 5.750     
                                                  0.333, 0.327, 0.321, 0.316, 0.311, 0.307, 0.303, 0.299, 0.296, 0.293, 0.291,  //6.00 --> 8.750
                                                  0.289, 0.287, 0.285, 0.284, 0.282, 0.281, 0.280, 0.280, 0.279, 0.279, 0.278,  //9.00 --> 11.75
                                                  0.278};                                                                       //12.0 --> 12.00

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
    public static MechanismLigament2d simulatorRoot;
    public static MechanismLigament2d realRoot;

    //Extras
    public double diffFactor = 1.0;
    public double pivotAngle = 0.0;

    public Shooter() {
        //Flywheel Initalization
        leftFlywheelMotor = new TalonFX(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID);
        rightFlywheelMotor = new TalonFX(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID);

        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);
        
        flywheelLeftBangController = new BangBangController();
        flywheelRightBangController = new BangBangController();

        flywheelFeedForwardLeft = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KV,
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KA);

        flywheelFeedForwardRight = new SimpleMotorFeedforward(RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KS, 
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KV,
                                                        RobotMap.Shooter.FLYWHEEL_FEED_FORWARD_KA);

        TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();

        flywheelConfigs.Slot0.kP = 0.3;
        flywheelConfigs.Slot0.kI = 40;
        flywheelConfigs.Slot0.kD = 0.0000;
        flywheelConfigs.Slot0.kV = 0;

        flywheelConfigs.Voltage.PeakForwardVoltage = 12;
        flywheelConfigs.Voltage.PeakReverseVoltage = -12;

        leftFlywheelMotor.getConfigurator().apply(flywheelConfigs);
        rightFlywheelMotor.getConfigurator().apply(flywheelConfigs);


        //Angle Initalization
        leftAngleMotor = new TalonFX(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID);
        rightAngleMotor = new TalonFX(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID);

        leftAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        rightAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        rightAngleMotor.setControl(new Follower(leftAngleMotor.getDeviceID(), true));

        angleFeedForward = new ArmFeedforward(RobotMap.Shooter.ANGLE_FEED_FORWARD_KS, 
                                              RobotMap.Shooter.ANGLE_FEED_FORWARD_KG, 
                                              RobotMap.Shooter.ANGLE_FEED_FORWARD_KV, 
                                              RobotMap.Shooter.ANGLE_FEED_FORWARD_KA);

        TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(Math.PI/2, 
            angleFeedForward.maxAchievableAcceleration(12, getAbsoluteShooterAngle(), leftAngleMotor.getVelocity().refresh().getValueAsDouble()));

        anglePIDController = new ProfiledPIDController(RobotMap.Shooter.ANGLE_PID_CONTROLLER_P, 
                                                       RobotMap.Shooter.ANGLE_PID_CONTROLLER_I,
                                                       RobotMap.Shooter.ANGLE_PID_CONTROLLER_D, 
                                                       pidConstraints);
    

        angleEncoder = new DutyCycleEncoder(RobotMap.Shooter.ANGLE_ENCODER_ID);

        //Limit Switch Initalization
        //angleLimitSwitch = new DigitalInput(RobotMap.Shooter.ANGLE_HARD_STOP_SWITCH_ID);

        //Interpolation Initalization
        initalizeInterpolationTable();

        //Current Limit Initalization
        CurrentLimitsConfigs clc40 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
        //CurrentLimitsConfigs clc30 = new CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30);
        
        leftFlywheelMotor.getConfigurator().apply(clc40);
        rightFlywheelMotor.getConfigurator().apply(clc40);

        leftAngleMotor.getConfigurator().apply(clc40);
        rightAngleMotor.getConfigurator().apply(clc40);

        resetFlywheelEncoders();

        //Simulation Initalization
        shooterSim = new Mechanism2d(300 / 33.07, 300 / 33.07);
        MechanismRoot2d shooterRoot = shooterSim.getRoot("Shooter", 
                                                         (RobotMap.Shooter.SIMULATION_OFFSET + 150) / RobotMap.Shooter.SIMULATION_SCALE,
                                                         (RobotMap.Shooter.SIMULATION_OFFSET + 150) / RobotMap.Shooter.SIMULATION_SCALE);
        
        simulatorRoot = shooterRoot.append(
                new MechanismLigament2d(
                        "Arm",
                        (double) (RobotMap.Shooter.SHOOTER_SIM_LENGTH) / RobotMap.Shooter.SIMULATION_SCALE,
                        0,
                        5.0,
                        new Color8Bit(255, 0, 0)));

        realRoot = shooterRoot.append(
                new MechanismLigament2d(
                        "Arm Real",
                        (double) (RobotMap.Shooter.SHOOTER_SIM_LENGTH) / RobotMap.Shooter.SIMULATION_SCALE,
                        0,
                        5.0,
                        new Color8Bit(255, 255, 0)));
    }


    //Flywheel Functions
    public double calculateFlywheelSpeedLeft(double speed) {
        final double bangOutput = flywheelLeftBangController.calculate(leftFlywheelMotor.getVelocity().refresh().getValueAsDouble(), (speed / (2 * Math.PI * 0.0508)));
        final double flywheelFeedForwardOutput = flywheelFeedForwardLeft.calculate(speed);

        
        DESIRED_LEFT_RPM_ENTRY.setDouble(speed / (2 * Math.PI * 0.0508) * 60);
        return (12 * bangOutput) + (flywheelFeedForwardOutput);
    }

    public double calculateFlywheelSpeedRight(double speed) {
        final double bangOutput = flywheelRightBangController.calculate(rightFlywheelMotor.getVelocity().refresh().getValueAsDouble(), (speed / (2 * Math.PI * 0.0508)));
        final double flywheelFeedForwardOutput = flywheelFeedForwardRight.calculate(speed);

        DESIRED_RIGHT_RPM_ENTRY.setDouble(speed / (2 * Math.PI * 0.0508) * 60);
        return (12 * bangOutput) + (flywheelFeedForwardOutput);

    }

    public double getVelocitySpeedLeft() {
        return leftFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }

    public double getVelocitySpeedRight() {
        return rightFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }


    //Angle Functions
    public double calculateAngleSpeed(double angle) {

        TrapezoidProfile.Constraints constrain = new TrapezoidProfile.Constraints(
			Math.PI/2, 
			angleFeedForward.maxAchievableAcceleration(12, getAbsoluteShooterAngle(), leftAngleMotor.getVelocity().refresh().getValueAsDouble())); // Change: Alan's job

		anglePIDController.setConstraints(constrain);

        final double angleOutput = anglePIDController.calculate(getAbsoluteShooterAngle(), angle);
        final double angleFeedforward = angleFeedForward.calculate(angle, anglePIDController.getSetpoint().velocity);
        return angleOutput + angleFeedforward;
    }

    public double getAbsoluteShooterAngle() {
        return angleEncoder.getAbsolutePosition();
    }

	public boolean atHardLimit() {
        return false;
        //return !angleLimitSwitch.get();
    }

    public void resetFlywheelEncoders() {
        leftFlywheelMotor.setPosition(0);
        rightFlywheelMotor.setPosition(0);
    }

    //Interpolation Functions
    public void initalizeInterpolationTable() {
        for(int i=0;i<interpolationAngles.length;i++){
            angleInterpolator.put((double)i*0.25+1.0,interpolationAngles[i]);
        }
        for(int i=0;i<interpolationTimes.length;i++){
            timeInterpolator.put((double)i*0.25+1.0,interpolationTimes[i]);
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
        ACTUAL_LEFT_RPM_ENTRY.setDouble(-getVelocitySpeedLeft() * 60);
        ACTUAL_RIGHT_RPM_ENTRY.setDouble(getVelocitySpeedRight() * 60);
        RPM_DIFF_FACTOR_ENTRY.setDouble(diffFactor);

        //Angle Entries
        ACTUAL_SHOOTER_ANGLE_ENTRY.setDouble(Math.toDegrees(getAbsoluteShooterAngle()));
        realRoot.setAngle(getAbsoluteShooterAngle());
        simulatorRoot.setAngle(pivotAngle);

        //Limit Entries
        ANGLE_LIMIT_SWITCH_STATUS_ENTRY.setBoolean(atHardLimit());

        SmartDashboard.putData("ShooterSim", shooterSim);
        Logger.recordOutput("ShooterMechanism", shooterSim);
    }
    
}