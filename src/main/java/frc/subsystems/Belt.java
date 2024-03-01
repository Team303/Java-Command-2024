package frc.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap.Intake;

public class Belt extends SubsystemBase {
    public final TalonFX beltMotor;

    public final TalonFX indexerMotor;

    public final VelocityVoltage flywheelVoltage = new VelocityVoltage(0, 0, true, 0, 0, false, false, false);

    public final CANSparkMax centerMotor;

    private final DigitalInput beam;
    public static final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("Amogh Belt"); // Shuffleboard tab
    public static final GenericEntry BELT_SPEED_ENTRY = INTAKE_TAB.add("Belt Speed", 0.0).getEntry();
    public static final GenericEntry BBC_ENTRY = INTAKE_TAB.add("BBC Status", true).getEntry();

    public Belt() {
        indexerMotor = new TalonFX(Intake.INDEX_MOTOR_ID);

        beam = new DigitalInput(Intake.BEAM_PORT);
        beltMotor = new TalonFX(Intake.BELT_MOTOR_ID);
        beltMotor.setInverted(true);
        beltMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration flywheelConfigs = new TalonFXConfiguration();

        flywheelConfigs.Slot0.kP = 0.3;
        flywheelConfigs.Slot0.kI = 40;
        flywheelConfigs.Slot0.kD = 0.0000;
        flywheelConfigs.Slot0.kV = 0;

        centerMotor = new CANSparkMax(Intake.CENTER_ID, MotorType.kBrushless);
        // rightCenterMotor = new CANSparkMax(Intake.RIGHT_CENTER_ID,
        // MotorType.kBrushless);

        flywheelConfigs.Voltage.PeakForwardVoltage = 12;
        flywheelConfigs.Voltage.PeakReverseVoltage = -12;

        beltMotor.getConfigurator().apply(flywheelConfigs);

        CurrentLimitsConfigs clc40 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
        beltMotor.getConfigurator().apply(clc40);
    }

    public void runBelt() {
        beltMotor.setControl(flywheelVoltage.withVelocity(-5500));
        Logger.recordOutput("belt speed", beltMotor.getVelocity().refresh().getValueAsDouble());
        // beltMotor.setVoltage(12);
        centerMotor.setVoltage(12);
        indexerMotor.setVoltage(10);
    }

    public void runBeltInReverse() {
        beltMotor.setControl(flywheelVoltage.withVelocity(5500));
        Logger.recordOutput("belt speed", beltMotor.getVelocity().refresh().getValueAsDouble());
        // beltMotor.setVoltage(12);
        centerMotor.setVoltage(-12);
        indexerMotor.setVoltage(-10);
    }

    public void runBelt(double belt, double center, double indexer) {
        beltMotor.setVoltage(belt);
        centerMotor.setVoltage(center);
        indexerMotor.setVoltage(indexer);
    }

    public void shoot() {
        indexerMotor.setVoltage(12);
    }

    public void stopMotors() {
        beltMotor.setVoltage(0);
        centerMotor.setVoltage(0);
        indexerMotor.setVoltage(0);
    }

    public boolean getBeam() {
        return beam.get();
    }

    @Override
    public void periodic() {
        BBC_ENTRY.setBoolean(getBeam());
    }

}
