package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climb implements Runnable, ServiceableModule {
    Thread mThread;
    
    InputManager mOperatorInputManager;

    //TalonSRX CBRMotor; 
    //TalonSRX CBLMotor; 

    TalonSRX CFRMotor; 
    TalonSRX CFLMotor; 

    TalonSRX LeftHookRotateMotor;

    TalonSRX RightHookRotateMotor;

    
    double catchPoint = -111; //point at which the lift is considered up and should begin bias power
    boolean frontState;
    boolean backState;
    boolean rotateSpeedState;

    double holdingBias = -0.15;
    double pullingPower = -0.5;
    
    double pushingPower = 0.2;
    double fastPushingPower = 0.5;

    double highPosition = 9000;

    public Climb(InputManager inputManager) {
        mOperatorInputManager = inputManager;
    }

    boolean autoConditionSatisfied;
    boolean autoEnabled;

    public boolean init() {
        CFLMotor = new TalonSRX(20);
        CFRMotor = new TalonSRX(21);
        //CBLMotor = new TalonSRX(22);
        //CBRMotor = new TalonSRX(23);
        LeftHookRotateMotor = new TalonSRX(24);
        RightHookRotateMotor = new TalonSRX(23);
        CFLMotor.setSelectedSensorPosition(0);
        CFRMotor.setSelectedSensorPosition(0);
        //CBLMotor.setSelectedSensorPosition(0);
        //CBRMotor.setSelectedSensorPosition(0);

        //CBLMotor.setInverted(true);
        System.out.println("[Climb] finished initialisation");
        return true;
    }

    void climbPID(double goal, TalonSRX motor) {
        double p = 0.0015;
        double distance = motor.getSelectedSensorPosition();
        
        double error = goal - distance;
        double output = error * p;
        double maxSpeed = 0.7;
        if (output > maxSpeed) {
            output = maxSpeed;
        } else if (output <= 0) {
            output = 0;
        }

        output = -output;
        motor.set(ControlMode.PercentOutput, output);
    }

    public boolean start() {
        boolean ret;

        ret = mOperatorInputManager.map();
        if(!ret) return ret;

        exitFlag = false;

        mThread = new Thread(this, "Climb");
        mThread.start();

        rotateSpeedState = false;

        return true;
    }

    public boolean stop() {
        if(mThread == null) return true;
        exitFlag = true;
        try {Thread.sleep(20);} catch (Exception e) {}

        if(mThread.isAlive()) {
            mThread.interrupt();
            try {Thread.sleep(20);} catch (Exception e) {}
            if(mThread.isAlive()) {
                return false;
            }
        }

        return true;
    }

    public void standby(boolean takeConfigOptions) {
        SmartDashboard.putNumber("CFL Current", CFLMotor.getStatorCurrent());
        SmartDashboard.putNumber("CFR Current", CFRMotor.getStatorCurrent());
        //SmartDashboard.putNumber("CBL Current", CBLMotor.getStatorCurrent());
        //SmartDashboard.putNumber("CBR Current", CBRMotor.getStatorCurrent());
        SmartDashboard.putNumber("CFR Position", CFRMotor.getSelectedSensorPosition());
        SmartDashboard.putNumber("CFL Position", CFLMotor.getSelectedSensorPosition());
    }

    public boolean exitFlag; //this flag is set true when the loop is to be exited

    int climbState = 0;
    public void advancedClimb() {
        switch (climbState) {
            case 0:
                // extend climb bars
                break;
            case 1:
                //rotate climb bars on to bar  
                break;
            case 2:
                //pull the 
                break;
            default:
                break;
        }
    }

    void extendRightArm() {
        if(CFRMotor.getSelectedSensorPosition() > highPosition) {
            CFRMotor.set(ControlMode.PercentOutput, pushingPower);
        } else {
            CFRMotor.set(ControlMode.PercentOutput, fastPushingPower);
        }
    }

    void extendLeftArm() {
        if(CFLMotor.getSelectedSensorPosition() > highPosition) {
            CFLMotor.set(ControlMode.PercentOutput, pushingPower);
        } else {
            CFLMotor.set(ControlMode.PercentOutput, fastPushingPower);
        }
    }

    void climbArms() {
        CFRMotor.set(ControlMode.PercentOutput, pullingPower);
        CFLMotor.set(ControlMode.PercentOutput, pullingPower);
    }

    void holdArms() {
        CFLMotor.set(ControlMode.PercentOutput, holdingBias);
        CFRMotor.set(ControlMode.PercentOutput, holdingBias);
    }

    void releaseArms() {
        CFLMotor.set(ControlMode.PercentOutput, 0);
        CFRMotor.set(ControlMode.PercentOutput, 0);
    }

    public void run() {
        exitFlag = false;
        backState = false;
        frontState = false;
        System.out.println("[Climb] entered independent service");
        while(!exitFlag) {
            
            if(mOperatorInputManager.getLeftTriggerDigital()){ //front pull 
                frontState = true;
                climbArms();
            } else if (mOperatorInputManager.getLeftBumper()) { //front release
                frontState = false;
                extendLeftArm();
                extendRightArm();
            } else if (frontState) { //apply bias
                holdArms();
            }
            else {
                CFLMotor.set(ControlMode.PercentOutput, 0);
                CFRMotor.set(ControlMode.PercentOutput, 0);
            }

            /*if(mOperatorInputManager.getRightTriggerDigital()){ //back pull 
                backState = true;
                //CBLMotor.set(ControlMode.PercentOutput, pullingPower);
                //CBRMotor.set(ControlMode.PercentOutput, pullingPower);
            } else if (mOperatorInputManager.getRightBumper()) { //back release
                backState = false;
                //CBLMotor.set(ControlMode.PercentOutput, pushingPower);
                CBRMotor.set(ControlMode.PercentOutput, pushingPower);
            } else if (backState) { //apply bias
                CBLMotor.set(ControlMode.PercentOutput, holdingBias);
                CBRMotor.set(ControlMode.PercentOutput, holdingBias);
            }
            else {
                CBLMotor.set(ControlMode.PercentOutput, 0);
                CBRMotor.set(ControlMode.PercentOutput, 0);
            }*/

            double rotateOutPower = (rotateSpeedState)? 0.75 : 0.20;

            double rotateInPower = (rotateSpeedState)? 0.80 : 0.25;

            if(mOperatorInputManager.getLeftStickX() > 0.5) {
                LeftHookRotateMotor.set(ControlMode.PercentOutput, rotateInPower);
                RightHookRotateMotor.set(ControlMode.PercentOutput, rotateInPower);
            } else if(mOperatorInputManager.getLeftStickX() < -0.5) {
                LeftHookRotateMotor.set(ControlMode.PercentOutput, -rotateOutPower);
                RightHookRotateMotor.set(ControlMode.PercentOutput, -rotateOutPower);
            } else {
                LeftHookRotateMotor.set(ControlMode.PercentOutput, 0);
                RightHookRotateMotor.set(ControlMode.PercentOutput, 0);
            }

            if(mOperatorInputManager.getRightBumperPressed()) {
                rotateSpeedState = !rotateSpeedState;
            }
            
            if(autoEnabled) {
                //climbPID(100, CBLMotor);
                //climbPID(100, CBRMotor);
                climbPID(200, CFLMotor);
                climbPID(200, CFRMotor);
                
            }

            try {Thread.sleep(50);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Climb] left independent service");

    }
}
