package frc.commands.shooter;

import static frc.robot.Robot.shooter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command; 

public class ShootIndexer extends Command {

    Timer timer = new Timer();
    double desiredVelocity = 0.0;

    public ShootIndexer(double height, double range) {
        addRequirements(shooter);
        double desiredAngle = Math.atan((2 * height) / range);
        desiredVelocity = Math.sqrt(2 * height * 9.8) / Math.sin(desiredAngle); //new equation
    }

    @Override
    public void initialize() {
        timer.restart();
    }

    @Override
    public void execute() {
        shooter.setIndexerSpeed(0.2);
        shooter.leftFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeed(desiredVelocity));
        shooter.rightFlywheelMotor.setVoltage(shooter.calculateFlywheelSpeed(desiredVelocity));
    } 

    @Override
    public boolean isFinished() {
        return /*shooter.getBeamBreak() == false ||*/ timer.hasElapsed(2.0);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setIndexerSpeed(0);
        timer.stop();
    }

}
