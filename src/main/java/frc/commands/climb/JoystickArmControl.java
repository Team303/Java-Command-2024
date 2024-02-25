package frc.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class JoystickArmControl extends Command {
    private final double deadband = 0.1;

    public JoystickArmControl() {
        addRequirements(Robot.climber);
    }

    @Override
    public void execute() {
        double armSpeed = -applyDeadband(Robot.controller.getRightY());

        if (armSpeed > 0) {
            if (!Robot.climber.isArmAtLimit(1) && !Robot.climber.isArmAtLimit(2)) {
                Robot.climber.extendArms(armSpeed);
            } else {
                Robot.climber.stopArms();
            }
        } else if (armSpeed < 0) {
            if (!Robot.climber.isArmAtLowerLimit(1) && !Robot.climber.isArmAtLowerLimit(2)) {
                Robot.climber.retractArms(armSpeed);
            } else {
                Robot.climber.stopArms();
            }
        } else {
            Robot.climber.stopArms();
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > deadband ? value : 0.0;
    }
}
