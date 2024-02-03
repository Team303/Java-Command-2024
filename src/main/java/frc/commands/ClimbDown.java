// package frc.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Robot;
// import frc.robot.RobotMap;

// public class ClimbDown extends Command {
//     public ClimbDown() {
//         addRequirements(Robot.climber);
//     }

//     @Override
//     public void initialize() {
//         // Initialization code goes here yk if u wanqt
//     }

//     @Override
//     public void execute() {
//         Robot.climber.retractArms(RobotMap.Swerve.SPEED);
//     }

//     @Override
//     public boolean isFinished() {
//         return Robot.climber.isArmOneAtLowerLimit() || Robot.climber.isArmTwoAtLowerLimit();
//     }

//     @Override
//     public void end(boolean interrupted) {
//         Robot.climber.stopArms();
//     }
// }
