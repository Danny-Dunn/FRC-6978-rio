// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C.Port;
import com.kauailabs.navx.frc.AHRS;

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
  LimelightController mLimelightController;


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
    mIntake = new Intake(mOperatorInputManager);
    mClimb = new Climb(mOperatorInputManager);
    mShooter = new Shooter(mDriverInputManager, mOperatorInputManager);
    mAutonomousController = new AutonomousController(mRealTimeDrive, mIntake, mShooter);
    mLimelightController = new LimelightController(mRealTimeDrive, mDriverInputManager);

    
    mRealTimeDrive.init();
    mIntake.init();
    mLimelightController.init();
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
    mLimelightController.standby(false);
  }

  //auto
  @Override
  public void autonomousInit() {
    mShooter.init();
    mRealTimeDrive.start();

    mIntake.start(true);
    mShooter.start(true);

    mAutonomousController.standby(true);

    mAutonomousController.init();
    mAutonomousController.start();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    mShooter.standby(true);
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    mClimb.init();
    mShooter.init();

    System.out.println("test?");
    
    mRealTimeDrive.start();
    mClimb.start();
    mIntake.start(false);
    mShooter.start();
    mLimelightController.start();
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
    mLimelightController.stop();
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
