package frc.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Robot;

public class TurnToAngle extends Command {

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
    

    public TurnToAngle(double angle) {
        addRequirements(Robot.swerve);

        controller = new PIDController(0.07, 0, 0.01);
        controller.enableContinuousInput(-180, 180);
        controller.setTolerance(2);

            boolean isBlue = true;

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
                isBlue = alliance.get() == DriverStation.Alliance.Blue;
            }

        this.angle = isBlue ? normalizeAngle(angle) : normalizeAngle(angle + 180);
    }

    @Override
    public void execute() {
        Robot.swerve.drive(new Translation2d(), -controller.calculate(normalizeAngle(Robot.navX.getAngle()), angle), true);
    }

    @Override
    public boolean isFinished() {
        return controller.atSetpoint() 
            || !(Robot.intake.getAbsolutePivotAngle() > Math.toRadians(45) && Robot.intake.getAbsolutePivotAngle() < Math.toRadians(180))
            || !(Robot.shooter.getAbsoluteShooterAngle() > Math.toRadians(350) || Robot.shooter.getAbsoluteShooterAngle() < Math.toRadians(10));
    }

    @Override
    public void end(boolean interrupted) {
        Robot.swerve.drive(new Translation2d(), 0, true);

    }
}
