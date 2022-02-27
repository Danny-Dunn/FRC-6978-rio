// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.Joystick;
import com.kauailabs.navx.frc.AHRS;

import java.lang.Thread;

//THIS class will handle threads and driverstation/FMS major events
public class Robot extends TimedRobot {
  
  InputManager mDriverInputManager;
  InputManager mOperatorInputManager;
  
  RealTimeDrive mRealTimeDrive;
  AutonomousController mAutonomousController;
  Intake mIntake;
  PneumaticController pneumatics;
  Climb mClimb;
  Shooter mShooter;


  AHRS navX;

  //SmartDashboard sdb = new SmartDashboard();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    mDriverInputManager = new InputManager(0);
    mOperatorInputManager = new InputManager(1);
    
    navX = new AHRS(Port.kMXP);
    mRealTimeDrive = new RealTimeDrive(mDriverInputManager, navX);
    mAutonomousController = new AutonomousController(mRealTimeDrive);
    mIntake = new Intake(mOperatorInputManager);
    mClimb = new Climb(mOperatorInputManager);
    mShooter = new Shooter(mDriverInputManager);

    
    mRealTimeDrive.init();
    mIntake.init();
    //pneumatics = new PneumaticController(driverStick);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    mRealTimeDrive.standby(true);
    mIntake.standby(true);
  }

  //auto
  @Override
  public void autonomousInit() {

    mRealTimeDrive.start();

    mIntake.start();
    mShooter.start();

    mAutonomousController.init();
    mAutonomousController.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mClimb.init();
    mShooter.init();

    System.out.println("test?");
    
    mRealTimeDrive.start();
    mClimb.start();
    mIntake.start();
    mShooter.start();
    //pneumatics_thread.start();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    mShooter.standby(true);
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    mRealTimeDrive.stop();
    mClimb.stop();
    mIntake.stop();
    mShooter.stop();
    mAutonomousController.stop();
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
