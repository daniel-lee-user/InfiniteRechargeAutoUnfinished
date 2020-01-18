package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class Turn extends CommandBase {
  double setpoint = 0;
  public Turn(double angle) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setpoint = angle;
    addRequirements(Robot.drive);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.resetAngle();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if (setpoint < 0) {
      Robot.drive.driveForward(0.5, -0.5);
    }
    if (setpoint > 0) {
      Robot.drive.driveForward(-0.5, 0.5);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.drive.getAngle() > (setpoint - 0.3) && Robot.drive.getAngle() < (setpoint + 0.3));
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}