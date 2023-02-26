package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.Joystick;

import java.nio.channels.ShutdownChannelGroupException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;


public class Robot extends TimedRobot {


  TalonFX leftDrive1;
  TalonFX leftDrive2;
  TalonFX rightDrive1;
  TalonFX rightDrive2;

  TalonFX liftMotor;

  TalonSRX armWheels;
  TalonSRX armMotor;
  TalonSRX armRotator;

  Joystick driveStick;
  Joystick operatorStick;

  AHRS navX;
  
  boolean isSticking;

  double angleP;
  double angleD;
  double roll;

  boolean brakeStatus;

  Compressor compressor;
  Solenoid solenoid; //exitflag is gone :)
  boolean compressorState = false;
  boolean solenoidState = true;
  boolean compState = true;

  //manual mode stuff
  boolean manualMode = true;

  //driving stuff
  boolean fastDriving = false;

  //setting the zero for the arm
  Timer timer;
  boolean zeroCompleted = false;
  boolean zeroSettingBOOL = false;
  double zeroSettingAmperage = 8;
  double zeroSettingTime = 0.2;
  double zeroSettingTimeFlag;

  double armMax;

  double armGoal = 0;
  double armMaxDistance = 13000; //from testing
  double maxLiftSpeedDown = -0.1;
  double maxLiftSpeedUp = 0.3;

  //this is the increment that the arm will go out by each time you click the button. ex 0 -> 10 -> 20 -> 10
  double armGoalIncrement = 0.10;
  double liftGoalIncrement = 0.10;
  
  //lift pid 
  private double maxPosition;
  private double liftP;
  private double liftI;
  private double liftD;

  //drive on graph stuff
  private double graphCurX;
  private double graphCurY;
  private double graphLastX;
  private double graphLastY;
  private double graphCurTheta;
  private double graphLastTheta;

  //Collection<TalonFX> _fxes =  { new TalonFX(1), new TalonFX(2), new TalonFX(3) };

  //auto Variables
  int autoStep = 0;
  int DOGStep = 0;

  @Override
  public void robotInit() {
    leftDrive1 = new TalonFX(1);
    leftDrive2 = new TalonFX(2);
    rightDrive1 = new TalonFX(3);
    rightDrive2 = new TalonFX(4);

    liftMotor = new TalonFX(5);

    //testMotor = new TalonSRX(24); //this was on the comp bot for the rotating arm
    armMotor = new TalonSRX(11);
    armMotor.setInverted(true);
    armMotor.setSensorPhase(true);

    armRotator = new TalonSRX(23);

    armWheels = new TalonSRX(24);

    rightDrive1.setInverted(true);
    rightDrive2.setInverted(true);

    leftDrive2.set(ControlMode.Follower, 1);
    rightDrive2.set(ControlMode.Follower, 3);

    driveStick = new Joystick(0);
    operatorStick = new Joystick(1);

    navX = new AHRS();

    navX.calibrate();

    isSticking = false;

    brakeStatus = true;

    compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    //solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0); //gear shift
    solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1); //grabber


    SmartDashboard.putNumber("angleP", angleP);
    SmartDashboard.putNumber("angleD", angleD);

    timer = new Timer();
    timer.start();

    //lft PID
    /* 
    liftP = 0.075;
    liftMotor.config_kP(0, liftP);
    liftI = 0.000002;
    liftMotor.config_kI(0, liftI);
    liftD = 23.0;
    liftMotor.config_kD(0, liftD);
    liftMotor.configOpenloopRamp(0.15);
    liftMotor.setSelectedSensorPosition(0);
    liftMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 8, 7, 0.05));
    liftMotor.configClosedLoopPeakOutput(0, 0.5);
    
    // end of pid*/
    maxPosition = 124000;
        //lift pid
    SmartDashboard.putNumber("Lift P", liftP);
    SmartDashboard.putNumber("Lift I", liftI);
    SmartDashboard.putNumber("Lift D", liftD);


    SmartDashboard.putNumber("Arm PID Increment %", 10);

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();

    angleP = SmartDashboard.getNumber("angleP", 0.00001);
    angleD = SmartDashboard.getNumber("angleD", 0);
    roll = navX.getRoll();

    SmartDashboard.putBoolean("BreakStatus", brakeStatus);
    SmartDashboard.putNumber("pitch", roll);
    SmartDashboard.putBoolean("sticking?", isSticking);

    SmartDashboard.putNumber("Arm Length POS", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Arm Length AMP", armMotor.getStatorCurrent());
    SmartDashboard.putNumber("Arm Length MAX", armMax);

    SmartDashboard.putNumber("Arm Rotation Position", kDefaultPeriod);

    //lift pid
    liftP = SmartDashboard.getNumber("Lift P", 0.075);
    liftI = SmartDashboard.getNumber("Lift I", 0.000002);
    liftD = SmartDashboard.getNumber("Lift D", 23.000000);
    //liftMotor.config_kP(0, liftP);
    //liftMotor.config_kI(0, liftI);
    //liftMotor.config_kD(0, liftD);

    SmartDashboard.putNumber("Arm PID GOAL", armGoal);
    armGoalIncrement = SmartDashboard.getNumber("Arm PID Increment %", 10)/100;
    liftGoalIncrement = SmartDashboard.getNumber("Lift PID Increment %", 10)/100;

    SmartDashboard.putBoolean("ManualMode", manualMode);
    if(driveStick.getRawButtonPressed(13)){
      manualMode = !manualMode;
    }


    //drive on graph stuff

    if (driveStick.getRawButtonPressed(9)){
      resetDriveOnGraph();
    }
    calculateDriveOnGraphPosition();

    SmartDashboard.putNumber("Graph X", graphCurX);
    SmartDashboard.putNumber("Graph Y", graphCurY);
    SmartDashboard.putNumber("Graph Θ", graphCurTheta);

    //SmartDashboard.putNumber("ArmP", 0);

  }

  private void resetDriveOnGraph(){
    graphCurX = 0;
    graphCurY = 0;
    graphLastX = 0;
    graphLastY = 0;
    graphCurTheta = 0;
    graphLastTheta = 0;

    leftDrive1.setSelectedSensorPosition(0);
    rightDrive1.setSelectedSensorPosition(0);
    navX.reset();
  }
  //final position --> faces that position and drives to it

  private void Draph(double goalX, double goalY, double ඞ){
    double xError = goalX - graphCurX;
    double yError = goalY - graphCurY;
    double ඞError = Math.atan(goalX/goalY) - graphCurTheta;
    double angleP = 0.0002;
    double driveP = 0.00001;
    double maxSpeed = 0.4;
    double angleOutput = ඞError*angleP;
    double driveOutput = Math.sqrt((xError*xError * xError > 0 ? 1 : -1) + yError*yError * yError > 0 ? 1 : -1) * driveP;
    if(Math.abs(angleOutput) > maxSpeed){
      if(angleOutput > 0){
        angleOutput = maxSpeed;
      }else{
        angleOutput = -maxSpeed;
      }
    }
    if(Math.abs(driveOutput) > maxSpeed){
      if(driveOutput > 0){
        driveOutput = maxSpeed;
      }else{
        driveOutput = -maxSpeed;
      }
    }
    switch(DOGStep){
      case 0:
        leftDrive1.set(ControlMode.PercentOutput, angleOutput);
        rightDrive1.set(ControlMode.PercentOutput, -angleOutput);
        if(graphCurTheta >= ඞ - 5 && graphCurTheta <= ඞ + 5){
          leftDrive1.set(ControlMode.PercentOutput, 0);
          rightDrive1.set(ControlMode.PercentOutput, -0);
          DOGStep++;
          break;
        }
      case 1:
      leftDrive1.set(ControlMode.PercentOutput, driveOutput);
      rightDrive1.set(ControlMode.PercentOutput, driveOutput);
      if(graphCurX >= goalX - 1000 && graphCurY >= goalY - 1000){
        leftDrive1.set(ControlMode.PercentOutput, 0);
        rightDrive1.set(ControlMode.PercentOutput, -0);
        DOGStep++;
        break;
      }
    }

    
  }
  private void calculateDriveOnGraphPosition(){
    double averageDisplacement = ((leftDrive1.getSelectedSensorPosition() - graphLastX) + (rightDrive1.getSelectedSensorPosition() - graphLastY))/2;
    double graphCurTheta = navX.getAngle();
    double deltaX = averageDisplacement * Math.sin((graphCurTheta + graphLastTheta)/2); 
    double deltaY = averageDisplacement * Math.cos((graphCurTheta + graphLastTheta)/2); 

    graphCurX += deltaX;
    graphCurY += deltaY;

    graphLastTheta = graphCurTheta;
    graphLastX = graphCurX;
    graphLastY = graphCurY;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    /* POTNENTIAL AUTO STEPS
     * Placing a block
     *  - Don't place a block (1)
     *  - bottom layer: push it in, regardless of piece type (2)
     *  - mid layer with cone: move lift up, extend arm a bit and release claw (3)
     *  - mid layer with cube: move lift up, extend arm a bit and drive arm wheels out (4)
     *  - top layer with cone: move lift up, rotate arm up, extend arm and release claw (5)
     *  - top layer with cube: move lift up, rotate arm up, extend arm and drive arm wheels out (6)
     * Moving out of community
     *  - Starting in middle of field: drive over the switch, then come back to balance (1)
     *  - starting near sides of field: drive around switch and come back to balance (2)
     * Balancing
     *  - Balance (1)
     *  - Don't balance, teammate is 1114 (2)
     * 
     * - We can create different combinations of auto steps, and call them based on the number 
     * (ex. 2-2-1 auto is scoring a bottom layer piece, driving around the switch and then balancing from the other side)
     */

    //2-1-1
    switch(autoStep){
      case 0:
        //drive forward
        double goal = 100000;
        double maxSpeed = 0.7;
        positionPID(goal, maxSpeed);
        if(goal - 1000 < leftDrive1.getSelectedSensorPosition() || leftDrive1.getSelectedSensorPosition() < goal + 1000){
          autoStep++;
          break; 
        }
        
      case 1:
        //claw wheels out
        double timerFlag = timer.get() + 1;
        armWheels.set(ControlMode.PercentOutput, 0.2);
        if(timer.get() > timerFlag){
          autoStep++;
          break;
        }
      case 2:
        //drive backwards over switch
        goal = -200000;
        maxSpeed = 0.8;
        positionPID(goal, maxSpeed);
        if(goal - 1000 < leftDrive1.getSelectedSensorPosition() || leftDrive1.getSelectedSensorPosition() < goal + 1000){
          autoStep++;
          break; 
        }
      case 3:
        //balance code
        if(Math.abs(navX.getRoll()) >= 3){
          isSticking = true;
        } 
        
        if(isSticking){
          balancingPID(-0.016, angleD, 0.4);
        } else {
          leftDrive1.set(ControlMode.PercentOutput, 0.4);
          rightDrive1.set(ControlMode.PercentOutput, 0.4);
        }
        autoStep++;
        break;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    double armP = SmartDashboard.getNumber("ArmP", 0);

    armMotor.config_kP(0, armP);
    brakeStatus = false;
    setBrakeMode();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
/* 
    if(driveStick.getRawButton(1)){
      
      if(Math.abs(navX.getRoll()) >= 3){
        isSticking = true;
      } 
      
      if(isSticking){
        balancingPID(-0.016, angleD, 0.4);
      } else {
        leftDrive1.set(ControlMode.PercentOutput, 0.4);
        rightDrive1.set(ControlMode.PercentOutput, 0.4);
      }
    }
    else {
      //drive();

      isSticking = false;
      if (brakeStatus && !isSticking){
        brakeStatus = false;
        setBrakeMode();
      }
    }
*/
    if(driveStick.getRawButtonPressed(10)){
      fastDriving = !fastDriving;
    }
    ////////////////////////
    //pneumatics 
    /////////////

    
    // Gear shift
    if(driveStick.getRawButtonPressed(2)){
      solenoidState = !solenoidState;
      solenoid.set(solenoidState);
    }

     
    // if(driveStick.getRawButtonPressed(3)){
    //     if(compState == false){
    //      compressor.disable();
    //      }
    //      else if(compState == true){
    //        compressor.enableDigital();
    //      } 
    //      compState = !compState;  
    // }
    //grabber wheels
     if(driveStick.getRawButton(3)){
      armWheels.set(ControlMode.PercentOutput, 0.4);
     }else if(driveStick.getRawButton(4)){
      armWheels.set(ControlMode.PercentOutput, -0.4);
     }else{
      if(!solenoidState){
        armWheels.set(ControlMode.PercentOutput, -0.1);
      }else{
        armWheels.set(ControlMode.PercentOutput, 0);
      }
    
      //lift pid
    }if(operatorStick.getRawButton(8)){
      liftPID(80); //keep in percent form, not decimal form 
    }else if(operatorStick.getRawButton(7)){
      liftPID(0);
    }else{
      liftMotor.set(ControlMode.PercentOutput, 0);
    }


     //////////////////
     //end of pneumatics */
     //////////////////

    /*if(driveStick.getRawButtonPressed(5) && driveStick.getRawButtonPressed(6)){
      brakeStatus = !brakeStatus;
      setBrakeMode();
    }*/

    //lift
    /*
    if(driveStick.getRawButton(1)) {
      liftMotor.set(ControlMode.Position, (int)(25000)); //pid low
    } 
    else if(driveStick.getRawButton(2)) {
      liftMotor.set(ControlMode.Position, (int)(105000));//0.65*maxPosition)); //pid high
    } 
    else{
      liftMotor.set(ControlMode.PercentOutput, 0);
    }*/

    if(driveStick.getRawButtonPressed(9) || driveStick.getRawButtonPressed(10)){
      //set the flags for the amp code
      zeroCompleted = false;
    }

    if (manualMode){
      if(operatorStick.getRawButton(5)){
        armMotor.set(ControlMode.PercentOutput, -0.2); // in
      } else if(operatorStick.getRawButton(6)) {
        armMotor.set(ControlMode.PercentOutput, 0.2); // out
      }
      else if(operatorStick.getRawButton(9)){
        zeroArm(true);
      }
      else if(operatorStick.getRawButton(10)){
        zeroArm(false);
      }
      else{
        armMotor.set(ControlMode.PercentOutput, 0);
      }

      //rotate
      if(operatorStick.getRawButton(1)){
        armRotator.set(ControlMode.PercentOutput, 0.1); // arm up
      }else if(operatorStick.getRawButton(3)){
        armRotator.set(ControlMode.PercentOutput, -0.1); //arm down
      }else{
        armRotator.set(ControlMode.PercentOutput, 0);
      }

      //lift
      if(driveStick.getRawButton(1)){
        liftMotor.set(ControlMode.PercentOutput, 0.1); //manual
      }
      else if(driveStick.getRawButton(2)){
        liftMotor.set(ControlMode.PercentOutput, -0.1); //mnual
      } 
      // */
    }
    else{ //not in manual mode

      //Arm In/Out
      if (operatorStick.getRawButtonPressed(6)){
        armGoal += (armGoalIncrement * armMaxDistance);
        armGoal = armGoal > armMaxDistance ? armMaxDistance : armGoal;
      }
      if(operatorStick.getRawButtonPressed(5)){
        armGoal -= (armGoalIncrement * armMaxDistance);
        armGoal = armGoal < 300 ? 300 : armGoal;
      }
      //Zero Arm
      if(operatorStick.getRawButton(9)){
        zeroArm(true);
      }
      else if(operatorStick.getRawButton(10)){
        zeroArm(false);
      }
      else{
        armMotor.set(ControlMode.Position, armGoal);
      }


    if(operatorStick.getRawButtonPressed(1)){
      armRotator.set(ControlMode.PercentOutput, 0.4);
    }
    else if(operatorStick.getRawButtonPressed(3)){
      armRotator.set(ControlMode.PercentOutput, 0);
    }

    }  
    
  }

  public void positionPID(double goal, double maxSpeed){
    double error = goal - leftDrive1.getSelectedSensorPosition();
    double p = 0.0001;
    double output = error * p;
    if(Math.abs(output) > maxSpeed){
      if(output > 0){
        output = maxSpeed;
      }if(output < 0){
        output = -maxSpeed;
      }
    }
    leftDrive1.set(ControlMode.PercentOutput, output);
    rightDrive1.set(ControlMode.PercentOutput, output);
  }

  public void liftPID(double percentGoal){
    double goal = maxPosition * percentGoal/100;
    double error = goal - liftMotor.getSelectedSensorPosition();
    double liftP = 0.0001;
    double output = error * liftP;
    if(output > maxLiftSpeedUp){
      output = maxLiftSpeedUp;
    }else if(output < maxLiftSpeedDown){
      output = maxLiftSpeedDown;
    }
    liftMotor.set(ControlMode.PercentOutput, output);
    

  }

  public void zeroArm(boolean findingMin){
    if (Math.abs(armMotor.getStatorCurrent()) > zeroSettingAmperage){
      if (zeroSettingBOOL == false){
        zeroSettingTimeFlag = timer.get() + zeroSettingTime;
      }
      zeroSettingBOOL = true;
    }
    else{
      zeroSettingTimeFlag = Double.MAX_VALUE;
      zeroSettingBOOL = false;
    }
    if (zeroSettingBOOL){
      if (timer.get() > zeroSettingTimeFlag){
        zeroCompleted = true;
        if (findingMin){
          armMotor.setSelectedSensorPosition(0);
        }
        else{
          armMax = armMotor.getSelectedSensorPosition();
        }
      }
    }
    if (!zeroCompleted){
      if (findingMin){
        armMotor.set(ControlMode.PercentOutput,-0.2);
      }
      else{
        armMotor.set(ControlMode.PercentOutput, 0.2);
      }
    }
    else{
      armMotor.set(ControlMode.PercentOutput,0);
    }
  }

  public void setBrakeMode(){
    if (brakeStatus == true){
      leftDrive1.setNeutralMode(NeutralMode.Brake);
      leftDrive2.setNeutralMode(NeutralMode.Brake);
      rightDrive1.setNeutralMode(NeutralMode.Brake);
      rightDrive2.setNeutralMode(NeutralMode.Brake);
    }
    else{
      leftDrive1.setNeutralMode(NeutralMode.Coast);
      leftDrive2.setNeutralMode(NeutralMode.Coast);
      rightDrive1.setNeutralMode(NeutralMode.Coast);
      rightDrive2.setNeutralMode(NeutralMode.Coast);
    }
  }
  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void balancingDistancePID(double goal, double P, double D){
    double position = (leftDrive1.getSelectedSensorPosition() + rightDrive1.getSelectedSensorPosition()) / 2;
    double error = goal - position;


    if(error <= goal - 1000){
      leftDrive1.set(ControlMode.PercentOutput, 0);
      leftDrive2.set(ControlMode.PercentOutput, 0);
      rightDrive1.set(ControlMode.PercentOutput, 0);
      rightDrive2.set(ControlMode.PercentOutput, 0);
    }
  }

    
   public void balancingPID(double P, double D, double max){
    double error = navX.getRoll();
    double demand = error * P;

    demand = (demand > max)? max:demand;
    demand = (demand < -max)? -max:demand;

    leftDrive1.set(ControlMode.PercentOutput, demand);
    rightDrive1.set(ControlMode.PercentOutput, demand);
  }

  public void drive(){
    double x;
    double y;
    double maxY = 1;

    //Negative X for 2023 practice bot, positive X for 2022 comp bot
    x = -driveStick.getRawAxis(0);
    y = (driveStick.getRawAxis(4) + 1 )/2 - (driveStick.getRawAxis(3) + 1 )/2 ;

    x = x * x *x; 
    if (!fastDriving){
      x = x/1.7;
      y = y/2;
    }
    if(y > maxY){
      y = maxY;
    }else if(y < -maxY){
      y = -maxY;
    }
    leftDrive1.set(ControlMode.PercentOutput, y + x );
    rightDrive1.set(ControlMode.PercentOutput, y - x);
  }
}