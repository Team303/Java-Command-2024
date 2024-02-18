package frc.subsystems;

import static frc.robot.Robot.indexerBelt;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotMap;

public class Belt extends SubsystemBase {
    public final TalonFX belt;
    public final TalonFX indexer;
    private final DigitalInput beamBBC; //beam break connection
    public static final ShuffleboardTab INTAKE_TAB = Shuffleboard.getTab("Amogh Belt"); // Shuffleboard tab
    public static final GenericEntry BELT_SPEED_ENTRY = INTAKE_TAB.add("Belt Speed", 0.0).getEntry();
    public static final GenericEntry INDEXER_SPEED_ENTRY = INTAKE_TAB.add("Indexer Speed", 0.0).getEntry();


    public Belt() {
        beamBBC = new DigitalInput(RobotMap.Intake.BEAM_PORT);
        belt = new TalonFX(RobotMap.Intake.BELT_MOTOR_ID);
		belt.setNeutralMode(NeutralModeValue.Coast);

        indexer = new TalonFX(RobotMap.Intake.INDEXER_MOTOR_ID);
        indexer.setNeutralMode(NeutralModeValue.Coast);

		CurrentLimitsConfigs clc40 = new CurrentLimitsConfigs().withStatorCurrentLimit(40).withSupplyCurrentLimit(40);
		belt.getConfigurator().apply(clc40);
        indexer.getConfigurator().apply(clc40);
        
    }

    public boolean getBBC() {
        return beamBBC.get();
    }

    @Override
    public void periodic() {
        BELT_SPEED_ENTRY.setDouble(indexerBelt.belt.getVelocity().refresh().getValueAsDouble());
        INDEXER_SPEED_ENTRY.setDouble(indexerBelt.indexer.getVelocity().refresh().getValueAsDouble());

    }
}
