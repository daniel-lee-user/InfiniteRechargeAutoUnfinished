package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorControl.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
/**
 * Add your docs here.
 */
public class DriveTrain extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  AHRS navx = new AHRS(Port.kMXP);
  WPI_TalonSRX left = new WPI_TalonSRX(2);
  WPI_TalonSRX right = new WPI_TalonSRX(3);
  DifferentialDrive d = new DifferentialDrive(left, right);
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
  public void driveForward(double lPow, double rPow) {
    d.tankDrive(lPow, rPow);
  }
  public double getAngle() {
    return navx.getAngle();
  }
  public void resetAngle() {
    navx.zeroYaw();
  }
  public void resetDistance(){
    left.setSelectedSensorPosition(0);
    right.setSelectedSensorPosition(0);
  }
  public double getDistance(){
    double leftD = left.getSelectedSensorPosition();
    double rightD = right.getSelectedSensorPosition();
    return (Math.abs(leftD)+Math.abs(rightD))/2;
  }
}