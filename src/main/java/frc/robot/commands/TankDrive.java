package frc.robot.commands;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class TankDrive extends CommandBase {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain drive;
    double setpoint = 0;
  public TankDrive(double distance) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    setpoint = distance;
    addRequirements();
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    Robot.drive.resetDistance();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    Robot.drive.driveForward(0.5, 0.5);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (Robot.drive.getDistance() > (setpoint));
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