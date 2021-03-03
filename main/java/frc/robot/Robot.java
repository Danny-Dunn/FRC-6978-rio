// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; TODO: SmartDashboard
import edu.wpi.first.wpilibj.Joystick;

import java.lang.Thread;

//THIS class will handle threads and driverstation/FMS major events
public class Robot extends TimedRobot {
  
  Joystick driverStick;
  
  RealTimeDrive RTDrive;
  AlignDriveCamera AlignDC;

  Thread RTDrive_thread;
  Thread AlignDC_thread;

  //SmartDashboard sdb = new SmartDashboard();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driverStick = new Joystick(0);
    RTDrive = new RealTimeDrive(driverStick);
    AlignDC = new AlignDriveCamera(RTDrive);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {}

  //auto
  @Override
  public void autonomousInit() {
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    RTDrive.setup();
    AlignDC.initCamera();
    
    RTDrive_thread = new Thread(RTDrive, "RTDrive");
    AlignDC_thread = new Thread(AlignDC, "AlignDC");
    RTDrive_thread.start();
    AlignDC_thread.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RTDrive.alignDCOK = (AlignDC.checkIn > (System.currentTimeMillis() - 800)); //if no response for 800
    try {Thread.sleep(10);} catch (InterruptedException ie) {} //chec 10 times per second
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    RTDrive.exitFlag = true;
    AlignDC.exitFlag = true;
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
