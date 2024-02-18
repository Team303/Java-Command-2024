package frc.commands.belt;
import static frc.robot.Robot.indexerBelt;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;
import static frc.subsystems.Belt.INDEXER_SPEED_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class IndexAndBelt extends Command{

    public IndexAndBelt() {
        addRequirements(indexerBelt);
    }

    @Override
    public void execute() {
        indexerBelt.belt.setVoltage(12);
        indexerBelt.indexer.setVoltage(12);
    }

    @Override
    public boolean isFinished() {
        return indexerBelt.getBBC();
    }    

    @Override
    public void end(boolean interrupted) {
        indexerBelt.belt.setVoltage(0);
        indexerBelt.indexer.setVoltage(0);
      
    }
}
