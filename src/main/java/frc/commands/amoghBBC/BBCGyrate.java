package frc.commands.amoghBBC;
import static frc.robot.Robot.DrakesSnake;
import static frc.subsystems.Belt.BELT_SPEED_ENTRY;
import static frc.subsystems.Belt.INDEXER_SPEED_ENTRY;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotMap;

public class BBCGyrate extends Command{

    public BBCGyrate() {
        addRequirements(DrakesSnake);

    }

    @Override
    public void execute() {
        DrakesSnake.beltBBC.setVoltage(12);
        DrakesSnake.aritraBBC.setVoltage(12);
    }

    @Override
    public boolean isFinished() {
        return DrakesSnake.getBBC();
    }    

    @Override
    public void end(boolean interrupted) {
        DrakesSnake.beltBBC.setVoltage(0);
        DrakesSnake.aritraBBC.setVoltage(0);
      
    }
}
