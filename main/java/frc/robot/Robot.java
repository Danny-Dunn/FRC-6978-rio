// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; //TODO: SmartDashboard
import edu.wpi.first.networktables.LogMessage;
import edu.wpi.first.wpilibj.Joystick;

import java.lang.Thread;

//THIS class will handle threads and driverstation/FMS major events
public class Robot extends TimedRobot {
  
  Joystick driverStick;
  
  RealTimeDrive RTDrive;
  AlignDriveCamera AlignDC;
  Intake Intak;
  PneumaticController pneumatics;

  Thread RTDrive_thread;
  Thread AlignDC_thread;
  Thread Intake_thread;
  Thread pneumatics_thread;

  //SmartDashboard sdb = new SmartDashboard();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driverStick = new Joystick(0);
    
    RTDrive = new RealTimeDrive(driverStick);
    AlignDC = new AlignDriveCamera(RTDrive, driverStick);
    Intak = new Intake(driverStick);
    //pneumatics = new PneumaticController(driverStick);
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
    driverStick = new Joystick(0);
    RTDrive.setup();
    Intak.init();
    AlignDC.initCamera();
    //pneumatics.init();

    System.out.println("test?");
    
    
    RTDrive_thread = new Thread(RTDrive, "RTDrive");
    Intake_thread = new Thread(Intak, "Intake");
    AlignDC_thread = new Thread(AlignDC, "AlignDC");
    //pneumatics_thread = new Thread(pneumatics, "Pneumatics");
    RTDrive_thread.start();
    AlignDC_thread.start();
    Intake_thread.start();
    //pneumatics_thread.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    RTDrive.alignDCOK = true; //if no response for 800
    SmartDashboard.putNumber("shooterVelocity", AlignDC.shooterMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("shooterCurrent", AlignDC.shooterMotor.getStatorCurrent());
    try {Thread.sleep(10);} catch (InterruptedException ie) {} //chec 10 times per second
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    RTDrive.exitFlag = true;
    AlignDC.exitFlag = true;
    Intak.exitFlag = true;
    //pneumatics.exitFlag = true;
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
