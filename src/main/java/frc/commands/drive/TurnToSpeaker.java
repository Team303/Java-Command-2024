package frc.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class TurnToSpeaker extends Command {

  double angle;
  PIDController controller;

  private double normalizeAngle(double angle) {
    angle %= 360;
    if (Math.abs(angle) < 180)
      return angle;
    else if (angle > 0)
      return angle - 360;
    else
      return angle + 360;
  }

  public TurnToSpeaker() {
    addRequirements(Robot.swerve);

    controller = new PIDController(0.07, 0, 0.01);
    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(2);
  }

  @Override
  public void initialize() {
    this.angle = normalizeAngle(Robot.swerve.calculateAngleSpeaker());
  }

  @Override
  public void execute() {
    Robot.swerve.drive(new Translation2d(), -controller.calculate(normalizeAngle(Robot.navX.getAngle()), angle), true);
  }

  @Override
  public boolean isFinished() {
    return controller.atSetpoint();// || Math.abs(normalizeAngle(Robot.navX.getAngle()) - angle) < 2;
  }

  @Override
  public void end(boolean interrupted) {
    Robot.swerve.drive(new Translation2d(), 0, true);

  }
}
