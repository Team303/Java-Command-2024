package frc.commands.amoghBBC;
import static frc.robot.Robot.belt;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;
import static frc.subsystems.Belt.INDEXER_SPEED_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class BBCGyrate extends Command{

    public BBCGyrate() {
        addRequirements(belt);

    }

    @Override
    public void execute() {
        belt.beltBBC.setVoltage(12);
        belt.aritraBBC.setVoltage(12);
    }

    @Override
    public boolean isFinished() {
        return belt.getBBC();
    }    

    @Override
    public void end(boolean interrupted) {
        belt.beltBBC.setVoltage(0);
        belt.aritraBBC.setVoltage(0);
      
    }
}
