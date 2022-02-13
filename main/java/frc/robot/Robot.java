// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SPI;
import com.kauailabs.navx.frc.AHRS;

import java.lang.Thread;

//THIS class will handle threads and driverstation/FMS major events
public class Robot extends TimedRobot {
  
  Joystick driverStick;
  Joystick operatorStick;
  
  RealTimeDrive RTDrive;
  AlignDriveCamera AlignDC;
  Intake Intak;
  PneumaticController pneumatics;
  Climb climb;
  Shooter shooter;

  Thread RTDrive_thread;
  Thread AlignDC_thread;
  Thread Intake_thread;
  Thread pneumatics_thread;
  Thread climb_thread;
  Thread shooter_thread;

  AHRS navX;

  //SmartDashboard sdb = new SmartDashboard();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    driverStick = new Joystick(0);
    operatorStick = new Joystick(1);
    
    navX = new AHRS(Port.kMXP);
    RTDrive = new RealTimeDrive(driverStick, navX);
    AlignDC = new AlignDriveCamera(RTDrive, driverStick);
    Intak = new Intake(driverStick);
    climb = new Climb(driverStick, operatorStick);
    shooter = new Shooter(driverStick);

    
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
    //Intak.init();
    AlignDC.initCamera();
    //pneumatics.init();
    climb.init();
    shooter.init();

    System.out.println("test?");
    
    
    RTDrive_thread = new Thread(RTDrive, "RTDrive");
    //Intake_thread = new Thread(Intak, "Intake");
    AlignDC_thread = new Thread(AlignDC, "AlignDC");
    climb_thread = new Thread(climb, "Climb");
    shooter_thread = new Thread(shooter, "Shooter");
    //pneumatics_thread = new Thread(pneumatics, "Pneumatics");
    RTDrive_thread.start();
    AlignDC_thread.start();
    climb_thread.start();
    shooter_thread.start();
    //Intake_thread.start();
    //pneumatics_thread.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //TODO: REMOVE SAFETY BYPASS, NOT PRODUCTION SAFE
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
    climb.exitFlag = true;
    shooter.exitFlag = true;
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
