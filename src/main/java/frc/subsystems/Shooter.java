
package frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Shooter extends SubsystemBase {

    // Flywheel stuff
    public final TalonFX leftFlywheelMotor;
    public final TalonFX rightFlywheelMotor;

    public final VelocityVoltage flywheelVoltageLeft = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);
    public final VelocityVoltage flywheelVoltageRight = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    // Angle stuff
    public final TalonFX leftAngleMotor;
    public final TalonFX rightAngleMotor;

    public final ProfiledPIDController anglePIDController;
    public final ArmFeedforward angleFeedForward;

    public final DutyCycleEncoder angleEncoder;

    // Limit Switch Stuff
    // public final DigitalInput angleLimitSwitch;

    // Interploation Stuff
    public InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap angleInterpolator17 = new InterpolatingDoubleTreeMap();

    public InterpolatingDoubleTreeMap timeInterpolator21 = new InterpolatingDoubleTreeMap();
    public InterpolatingDoubleTreeMap timeInterpolator17 = new InterpolatingDoubleTreeMap();

    // Interpolation Table 1: 21.27 meters per second 💀
    public static double[] interpolationAngles21 = { 0.95, 0.95, 0.95, 0.95, 0.938, 0.883, 0.835, 0.794, 0.759, 0.727,
            0.700, // 1.00 --> 3.750
            0.676, 0.654, 0.635, 0.619, 0.604, 0.590, 0.578, 0.567, 0.557, 0.549, 0.541, // 4.00 --> 5.750
            0.533, 0.527, 0.521, 0.516, 0.511, 0.507, 0.503, 0.499, 0.946, 0.493, 0.491, // 6.00 --> 8.750
            0.489, 0.487, 0.485, 0.484, 0.484, 0.481, 0.480, 0.480, 0.479, 0.479, 0.478, // 9.00 --> 11.75
            0.478 }; // 12.0 --> 12.00
    // Interpolation Table 2: 17 meters per second
    // public static double[] interpolationAngles17 = {1.0642, 0.9670, 0.8827,
    // 0.8103, 0.7481, 0.6949, 0.6491, 0.6097, 0.5756, 0.5461, 0.5204, //1.00 -->
    // 3.750
    // 0.4980, 0.4784, 0.4612, 0.4460, 0.4327, 0.4210, 0.4107, 0.4015, 0.3935,
    // 0.3864, 0.3802, //3.00 --> 5.750
    // 0.3748, 0.3701, 0.3660, 0.3625, 0.3595, 0.3569, 0.3548, 0.3531, 0.3518,
    // 0.3508, 0.3501, //6.00 --> 8.750
    // 0.3497, 0.3496, 0.3497, 0.3501, 0.3507, 0.3515, 0.3525, 0.3537, 0.3550,
    // 0.3565, 0.3582, //9.00 --> 11.75
    // 0.3601}; //12.0 --> 12.00

    public static double[] interpolationTimes21 = new double[interpolationAngles21.length];
    // public static double[] interpolationTimes17 = new
    // double[interpolationAngles17.length];

    // Shuffleboard Stuff
    public static final ShuffleboardTab SHOOTER_TAB = Shuffleboard.getTab("Shooter"); // Shuffleboard tab

    // Flywheel Entries
    public static final GenericEntry DESIRED_LEFT_RPM_ENTRY = SHOOTER_TAB.add("Desired Left RPM", 0.0)
            .withPosition(0, 0).withSize(2, 1).getEntry();
    public static final GenericEntry DESIRED_RIGHT_RPM_ENTRY = SHOOTER_TAB.add("Desired Right RPM", 0.0)
            .withPosition(0, 2).withSize(2, 1).getEntry();
    public static final GenericEntry ACTUAL_LEFT_RPM_ENTRY = SHOOTER_TAB.add("Actual Left RPM", 0.0).withPosition(1, 0)
            .withSize(2, 1).getEntry();
    public static final GenericEntry ACTUAL_RIGHT_RPM_ENTRY = SHOOTER_TAB.add("Actual Right RPM", 0.0)
            .withPosition(1, 2).withSize(2, 1).getEntry();
    public static final GenericEntry RPM_DIFF_FACTOR_ENTRY = SHOOTER_TAB.add("RPM Diff Factor", 0.0).withPosition(2, 0)
            .withSize(1, 1).getEntry();
    public static final GenericEntry LEFT_FLYWHEEL_VOLTAGE_ENTRY = SHOOTER_TAB.add("Left Flywheel Voltage", 0.0)
            .withPosition(3, 0).withSize(2, 1).getEntry();
    public static final GenericEntry RIGHT_FLYWHEEL_VOLTAGE_ENTRY = SHOOTER_TAB.add("Right Flywheel Voltage", 0.0)
            .withPosition(3, 2).withSize(2, 1).getEntry();
    public static final GenericEntry LEFT_PIVOT_VOLTAGE_ENTRY = SHOOTER_TAB.add("Left Flywheel Pivot", 0.0)
            .withPosition(3, 0).withSize(2, 1).getEntry();
    public static final GenericEntry RIGHT_PIVOT_VOLTAGE_ENTRY = SHOOTER_TAB.add("Right Flywheel Pivot", 0.0)
            .withPosition(3, 2).withSize(2, 1).getEntry();

    public static final GenericEntry ANGLE_ENTRY_PIVOT = SHOOTER_TAB.add("Angle Entry Pivot", 0.0).getEntry();
    // Angle Entries
    public static final GenericEntry INTERPOLATED_DEGREES_ENTRY = SHOOTER_TAB.add("Desired Degrees", 0.0)
            .withPosition(2, 1).withSize(1, 1).getEntry();
    public static final GenericEntry ACTUAL_SHOOTER_ANGLE_ENTRY = SHOOTER_TAB.add("Actual Degrees", 0.0)
            .withPosition(5, 0).getEntry();

    // Limit Entries
    public static final GenericEntry ANGLE_LIMIT_SWITCH_STATUS_ENTRY = SHOOTER_TAB.add("Angle Hard Limit", false)
            .withPosition(0, 0).getEntry();

    // Extras
    public double diffFactor = 0.8;
    public double pivotAngle = 0.0;

    public Shooter() {
        // Flywheel Initalization

        // ayushBBC = new DigitalInput(8);

        leftFlywheelMotor = new TalonFX(RobotMap.Shooter.LEFT_FLYWHEEL_MOTOR_ID);
        rightFlywheelMotor = new TalonFX(RobotMap.Shooter.RIGHT_FLYWHEEL_MOTOR_ID);

        leftFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);
        rightFlywheelMotor.setNeutralMode(NeutralModeValue.Coast);

        leftFlywheelMotor.setInverted(false);
        rightFlywheelMotor.setInverted(true);

        TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();

        flywheelConfigs.Slot0.kP = 0.3;
        flywheelConfigs.Slot0.kI = 40;
        flywheelConfigs.Slot0.kD = 0.0000;
        flywheelConfigs.Slot0.kV = 0;

        flywheelConfigs.Voltage.PeakForwardVoltage = 12;
        flywheelConfigs.Voltage.PeakReverseVoltage = -12;

        leftFlywheelMotor.getConfigurator().apply(flywheelConfigs);
        rightFlywheelMotor.getConfigurator().apply(flywheelConfigs);

        MotorOutputConfigs alanbbc = new MotorOutputConfigs();
        alanbbc.withInverted(InvertedValue.Clockwise_Positive);
        rightFlywheelMotor.getConfigurator().apply(alanbbc);

        MotorOutputConfigs aritrabbc = new MotorOutputConfigs();
        aritrabbc.withInverted(InvertedValue.Clockwise_Positive);

        TalonFXConfiguration angleConfigs = new TalonFXConfiguration();

        angleConfigs.Slot0.kP = 0.05;
        angleConfigs.Slot0.kI = 0;
        angleConfigs.Slot0.kD = 0.0000;
        angleConfigs.Slot0.kV = 0;

        angleConfigs.Voltage.PeakForwardVoltage = 12;
        angleConfigs.Voltage.PeakReverseVoltage = -12;

        // Angle Initalization
        leftAngleMotor = new TalonFX(RobotMap.Shooter.LEFT_ANGLE_MOTOR_ID);
        rightAngleMotor = new TalonFX(RobotMap.Shooter.RIGHT_ANGLE_MOTOR_ID);

        leftAngleMotor.getConfigurator().apply(angleConfigs);

        leftAngleMotor.setNeutralMode(NeutralModeValue.Brake);
        rightAngleMotor.setNeutralMode(NeutralModeValue.Brake);

        rightAngleMotor.setControl(new Follower(leftAngleMotor.getDeviceID(), true));

        angleFeedForward = new ArmFeedforward(RobotMap.Shooter.ANGLE_FEED_FORWARD_KS,
                RobotMap.Shooter.ANGLE_FEED_FORWARD_KG,
                RobotMap.Shooter.ANGLE_FEED_FORWARD_KV,
                RobotMap.Shooter.ANGLE_FEED_FORWARD_KA);

        // go up to 90 degrees in one second 1/4 second to accelerate to max speed
        TrapezoidProfile.Constraints pidConstraints = new TrapezoidProfile.Constraints(Math.PI,
                Math.PI * 4);

        anglePIDController = new ProfiledPIDController(RobotMap.Shooter.ANGLE_PID_CONTROLLER_P,
                RobotMap.Shooter.ANGLE_PID_CONTROLLER_I,
                RobotMap.Shooter.ANGLE_PID_CONTROLLER_D,
                pidConstraints);

        anglePIDController.enableContinuousInput(0, 2 * Math.PI);
        anglePIDController.setTolerance(Math.toRadians(2.0));

        angleEncoder = new DutyCycleEncoder(RobotMap.Shooter.ANGLE_ENCODER_ID);

        // Limit Switch Initalization
        // angleLimitSwitch = new
        // DigitalInput(RobotMap.Shooter.ANGLE_HARD_STOP_SWITCH_ID);

        // Interpolation Initalization
        initalizeInterpolationTable21();
        // initalizeInterpolationTable17();

        // Current Limit Initalization
        CurrentLimitsConfigs clc40 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
        // CurrentLimitsConfigs clc30 = new
        // CurrentLimitsConfigs().withStatorCurrentLimit(30).withSupplyCurrentLimit(30);

        leftFlywheelMotor.getConfigurator().apply(clc40);
        rightFlywheelMotor.getConfigurator().apply(clc40);

        leftAngleMotor.getConfigurator().apply(clc40);
        leftAngleMotor.getConfigurator().apply(aritrabbc);
        rightAngleMotor.getConfigurator().apply(clc40);

        anglePIDController.reset(0);

    }

    // Flywheel Functions
    public double getVelocitySpeedLeft() {

        // return speed in rot/sec

        return leftFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }

    public double getVelocitySpeedRight() {
        // return speed in rot/sec
        return rightFlywheelMotor.getVelocity().refresh().getValueAsDouble();
    }

    // public boolean atShooterLimit() {
    // return ayushBBC.get();
    // }

    // Angle Functions
    public double calculateAngleSpeed(double angleRad) {
        if (angleRad > Math.PI * 2)
            throw new IllegalArgumentException("Radians  bitch");

        final double angleOutput = anglePIDController.calculate(getAbsoluteShooterAngle(), angleRad);
        final double angleFeedforward = angleFeedForward.calculate(angleRad, anglePIDController.getSetpoint().velocity);
        return angleOutput + angleFeedforward;
    }

    public boolean atSetpoint() {
        return anglePIDController.atSetpoint();
    }

    private double normalizeAngle(double angleRad) {
        angleRad %= Math.PI * 2;

        if (angleRad < 0)
            angleRad += Math.PI * 2;

        return angleRad;
    }

    public double getAbsoluteShooterAngle() {

        // get angular position in rad

        return normalizeAngle(angleEncoder.getAbsolutePosition() * 2 * Math.PI - Math.toRadians(140 + 205));

    }

    public double getAngleMotorVelocity() {

        // get angular velocity in rad/sec

        return leftAngleMotor.getVelocity().refresh().getValueAsDouble() * RobotMap.Shooter.ANGLE_CONVERSION_FACTOR * 2
                * Math.PI;
    }

    public boolean atHardLimit() {
        return false;
        // return !angleLimitSwitch.get();
    }

    // Interpolation Functions
    public void initalizeInterpolationTable21() {
        for (int i = 0; i < interpolationAngles21.length; i++) {
            angleInterpolator.put((double) i * 0.25 + 1.0, interpolationAngles21[i]);
        }
        for (int i = 0; i < interpolationTimes21.length; i++) {
            timeInterpolator21.put((double) i * 0.25 + 1.0, interpolationTimes21[i]);
        }
    }

    // public void initalizeInterpolationTable17() {
    // for(int i=0;i<interpolationAngles17.length;i++){
    // angleInterpolator17.put((double)i*0.25+1.0,interpolationAngles17[i]);
    // }
    // for(int i=0;i<interpolationTimes17.length;i++){
    // timeInterpolator17.put((double)i*0.25+1.0,interpolationTimes17[i]);
    // }
    // }

    public double interpolateAngle(double range) {

        return angleInterpolator.get(range);
    }

    public double interpolateTime(double range) {

        return timeInterpolator21.get(range);
    }

    // Factor Functions
    public double getFactor() {
        return diffFactor;
    }

    public void setFactor(double factor) {
        diffFactor = factor;
    }

    @Override
    public void periodic() {
        // Flywheel Entries

        Logger.recordOutput("left angle position", getAbsoluteShooterAngle() * (180 / Math.PI));

        // scale by 60 for rpm
        ACTUAL_LEFT_RPM_ENTRY.setDouble(-getVelocitySpeedLeft() * 60);
        ACTUAL_RIGHT_RPM_ENTRY.setDouble(getVelocitySpeedRight() * 60);
        LEFT_FLYWHEEL_VOLTAGE_ENTRY.setDouble(leftFlywheelMotor.getMotorVoltage().getValueAsDouble());
        RIGHT_FLYWHEEL_VOLTAGE_ENTRY.setDouble(rightFlywheelMotor.getMotorVoltage().getValueAsDouble());
        LEFT_PIVOT_VOLTAGE_ENTRY.setDouble(leftAngleMotor.getMotorVoltage().getValueAsDouble());
        RIGHT_PIVOT_VOLTAGE_ENTRY.setDouble(rightAngleMotor.getMotorVoltage().getValueAsDouble());

        RPM_DIFF_FACTOR_ENTRY.setDouble(diffFactor);

        // Angle Entries
        ACTUAL_SHOOTER_ANGLE_ENTRY.setDouble(getAbsoluteShooterAngle() * (180 / Math.PI));

        // Limit Entries
        ANGLE_LIMIT_SWITCH_STATUS_ENTRY.setBoolean(atHardLimit());
        ANGLE_ENTRY_PIVOT.setDouble(getAbsoluteShooterAngle());
    }

}
