//This file will work similarly to RTDrive

package frc.robot;
//Cross The Road Electronics(CTRE) libs must be installed
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; TODO: SmartDashboard
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;

public class AutoKernel implements Runnable {
  TalonFX DL1Motor; //DriveLeft1Motor
  TalonFX DL2Motor;
  TalonFX DR1Motor; //DriveRight1Motor
  TalonFX DR2Motor;
  public TalonSRX shooterMotor;
  TalonSRX loaderMotor;
  
  public string movementMode; //use this to select what type of movement you want the robot to perform
  
  public AutoKernel() {
    
  }
  
  public boolean setup() {
    DL1Motor = new TalonFX(0);
    DL2Motor = new TalonFX(1);
    DR1Motor = new TalonFX(2);
    DR2Motor = new TalonFX(3);
  }
  
  void setMotors(float rightInput, float leftInput) { //set all motors at once
    DL1Motor.set(leftInput);
    DL2Motor.set(leftInput);
    DR1Motor.set(rightInput); 
    DR2Motor.set(rightInput);
  }
  
  public boolean exitFlag;
  public void run() {
    //loop code here
    switch (movementMode) { //change between a few different modes of movement
      case "static":
        setMotors(0, 0);
        break;
      case "turn":
        //this is where PID code for gyro alignment can go
        break;
      default:
        break;
    }
  }
}
