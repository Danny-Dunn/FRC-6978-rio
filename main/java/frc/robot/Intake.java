package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Runnable, ServiceableModule{
    private Thread mThread;
    private boolean auto;
    private boolean hasRun;

    InputManager mOperatorInputManager;

    TalonSRX intakeLiftMotor; //retracts ball pickup structure into robot
    TalonSRX intakeRollerMotor; //drives rollers to pick up cargo

    public static class IntakeConfig {
        public static double deadZone = 0.1;
        public static double intakeFineAdjustSpeed = 0.5;
        public static boolean intakeFineAdjustEnabled = true;
        public static boolean rollerFineAdjustEnabled = false;
        public static double rollerOutSpeed = -0.35;
        public static double rollerInSpeed = 0.45;
        public static RollerMode defaultRollerMode = RollerMode.direct;
        public static LiftMode defaultLiftMode = LiftMode.position;
    }

    enum RollerMode {
        direct,
        none,
    };

    enum LiftMode {
        position,
        direct,
        none,
    };

    RollerMode mRollerMode;
    LiftMode mLiftMode;

    private boolean liftConditionSatisfied;
    private boolean rollerConditionSatisfied;

    boolean targetState = true; //determined by joystick buttons
    double intakeTargetPosition;
    double rollerPower;
    double liftOut;

    NetworkTable simTable;

    double intakeFullOutPosition = 1100;
    static double intakeParkPosition = -463;
    double intakeTicksPerRadian = 651.8986469;
    double intakeUpperPosition = -463; // should be -82
    double intakeHorizontalBias = 0.10;

   

    public boolean getRollerConditionSatisfied() {
        return rollerConditionSatisfied;
    }
    
    public static double getParkPosition() {
        return intakeParkPosition;
    }

    public boolean getLiftConditionSatisfied() {
        return liftConditionSatisfied;
    }

    public void setLift(double setpoint) {
        liftConditionSatisfied = false;
        intakeTargetPosition = setpoint;
        System.out.println("Set intake lift to " + setpoint);
    }

    public void setRollers(RollerMode mode, double power) {
        rollerConditionSatisfied = false;
        mRollerMode = mode;
        rollerPower = power;
    }

    public Intake(InputManager inputManager) {
        mOperatorInputManager = inputManager;
        mRollerMode = RollerMode.none;
        mLiftMode = LiftMode.none;
        hasRun = false;
    }

    public void simOut(String tag, Double value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setDouble(value);
        } else {
            SmartDashboard.putNumber(tag, value);
        }
    }

    public boolean init() {
        intakeLiftMotor = new TalonSRX(30);
        intakeRollerMotor = new TalonSRX(31);
        intakeRollerMotor.setInverted(false);
        intakeLiftMotor.setSelectedSensorPosition(intakeUpperPosition);
        intakeLiftMotor.setSensorPhase(true);
        intakeLiftMotor.setInverted(true);
        if(!RobotBase.isReal()) simTable = NetworkTableInstance.getDefault().getTable("simTable"); //simulation dummy outputs

        intakeTargetPosition = intakeParkPosition;

        System.out.println("[Intake] finished initialisation");
        return true;
    }

    public boolean start() {
        return start(false);
    }

    public void calibrateIntake() {
        System.out.println("[Intake] Pulling mechanism for calibration");
        intakeLiftMotor.set(ControlMode.PercentOutput, -0.2);
        try {Thread.sleep(500);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        intakeLiftMotor.setSelectedSensorPosition(intakeUpperPosition);
        System.out.println("[Intake] Finished encoder calibration");
        intakeLiftMotor.set(ControlMode.PercentOutput, 0);
    }

    public boolean start(boolean auto) {
        boolean ret;

        if(!auto) {
            ret = mOperatorInputManager.map();
            if(!ret) return ret;
        }

        exitFlag = false;
        
        this.auto = auto;

        if(!hasRun)
            calibrateIntake();
        
        targetState = false;

        mLiftMode = IntakeConfig.defaultLiftMode;
        mRollerMode = IntakeConfig.defaultRollerMode;

        mThread = new Thread(this, "Intake");
        mThread.start();

        hasRun = true;
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
        SmartDashboard.putNumber("intakeRadians", intakeLiftMotor.getSelectedSensorPosition() / intakeTicksPerRadian);
        SmartDashboard.putNumber("intakePower", intakeLiftMotor.getStatorCurrent() * intakeLiftMotor.getBusVoltage()); //calculate the total power draw in watts
        SmartDashboard.putNumber("intakeTargetRadians", intakeTargetPosition / intakeTicksPerRadian);
        SmartDashboard.putNumber("intakeTicks", intakeLiftMotor.getSelectedSensorPosition());
        SmartDashboard.putString("rollerMode", mRollerMode.toString());
        SmartDashboard.putString("lfitMode", mLiftMode.toString());
        SmartDashboard.putNumber("intakeOut", liftOut);
    }

    long integralTS;
    double integralError;

    double intakeLiftPID(double target, double kP, double kI) {
        double error = target - intakeLiftMotor.getSelectedSensorPosition();

        double weightBias = Math.sin(intakeLiftMotor.getSelectedSensorPosition() / intakeTicksPerRadian) * intakeHorizontalBias;

        integralError += (error * (System.currentTimeMillis() - integralTS)) / 10000000000d;

        double movePower = (error * kP);
        movePower = (movePower > 0.28)? 0.28 : movePower;
        movePower = (movePower < -0.065)? -0.065 : movePower;
        SmartDashboard.putNumber("Move Power", movePower);
        SmartDashboard.putNumber("Intake Error", error);

        return weightBias + movePower /*+ (integralError * kI)*/;
    }

    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        intakeTargetPosition = intakeParkPosition;
        long lastCalcTS = System.nanoTime();
        System.out.println("[Intake] entered independent service");
        if(!auto) {
            mOperatorInputManager.getRightBumperPressed();
            mOperatorInputManager.getSouthButtonPressed();
            mOperatorInputManager.getSouthButtonReleased();
            mOperatorInputManager.getNorthButtonReleased();
            mOperatorInputManager.getNorthButtonPressed();
        }
        while(!exitFlag) {
            double rollerOut = 0.0;
            
            if(!auto) {
                if(mOperatorInputManager.getPOV() == 90) {
                    mLiftMode = (mLiftMode.equals(LiftMode.direct))? LiftMode.position : LiftMode.direct;
                }
                
                if(mOperatorInputManager.getSouthButtonPressed()) {
                    setRollers(RollerMode.direct, IntakeConfig.rollerInSpeed);
                } else if(mOperatorInputManager.getNorthButtonPressed()) {
                    setRollers(RollerMode.direct, IntakeConfig.rollerOutSpeed);
                } else if (mOperatorInputManager.getSouthButtonReleased()) {
                    setRollers(RollerMode.direct, 0);
                } else if(mOperatorInputManager.getNorthButtonReleased()) {
                    setRollers(RollerMode.direct, 0);
                }
                /*if(mOperatorInputManager.getRightTriggerDigital()) {
                    calibrateIntake();
                }*/
            }
            
            switch (mRollerMode) {
                case direct:
                    rollerConditionSatisfied = true;
                    rollerOut = rollerPower;
                    break;
                default:
                    rollerOut = 0;
                    break;
            }

            /*switch(mIntakeMode) {
                case user:
                    


                    
                    break;
                case rollers:
                    autoConditionSatisfied = true;
                    rollerOut = rollerPower;
                    break;
                case position:
                    autoConditionSatisfied = Math.abs(intakeTargetPosition - intakeLiftMotor.getSelectedSensorPosition()) < 100;
                    break;
                case all:
                    if(Math.abs(intakeTargetPosition - intakeLiftMotor.getSelectedSensorPosition()) < 100) {
                        autoConditionSatisfied = true;
                    }
                    rollerOut = rollerPower;
                    break;
                case none:
                    intakeTargetPosition = intakeParkPosition;
                    break;
            }*/

            switch (mLiftMode) {
                case position:
                    if(!auto) {
                        double deltaT = (System.nanoTime() - lastCalcTS) / 1000000.0d;
                        lastCalcTS = System.nanoTime();

                        if(IntakeConfig.intakeFineAdjustEnabled) {
                            double y = -mOperatorInputManager.getLeftStickY();
                            y = (y < IntakeConfig.deadZone && y > -IntakeConfig.deadZone)? 0 : y;
                            if(y != 0.0) y = (y > 0.0)? y - IntakeConfig.deadZone : y + IntakeConfig.deadZone; //eliminate jump behaviour
                            y = y / (1 - IntakeConfig.deadZone); 

                            //intakeTargetPosition += y * deltaT * IntakeConfig.intakeFineAdjustSpeed;
                        }
                        
                        if(mOperatorInputManager.getPOV() == 0) { //set the intake forward
                            setLift(intakeFullOutPosition);
                        } else if(mOperatorInputManager.getPOV() == 180) {
                            setLift(intakeParkPosition);
                        }
                    }

                    intakeTargetPosition = (intakeTargetPosition > intakeFullOutPosition)? intakeFullOutPosition : intakeTargetPosition;
                    intakeTargetPosition = (intakeTargetPosition < intakeParkPosition)? intakeParkPosition : intakeTargetPosition;            

                    liftConditionSatisfied = Math.abs(intakeTargetPosition - intakeLiftMotor.getSelectedSensorPosition()) < 100;

                    liftOut = -intakeLiftPID(intakeTargetPosition, -0.0025, 0);
                    liftOut = (liftOut > 0.19)? 0.19 : liftOut;
                    liftOut = (liftOut < -0.29)? -0.29 : liftOut;

                    if(intakeTargetPosition == intakeParkPosition && intakeLiftMotor.getSelectedSensorPosition() - intakeTargetPosition < 350) {
                        liftOut = -0.07;
                    } else if(intakeTargetPosition == intakeFullOutPosition && intakeLiftMotor.getSelectedSensorPosition() - intakeParkPosition < 500) {
                        liftOut = 0.40;
                    } else if(intakeTargetPosition == intakeFullOutPosition)  {
                        if(intakeLiftMotor.getSelectedSensorPosition() - intakeFullOutPosition <= 400) {
                            liftOut = 0.07;
                        }
                    } else if(intakeTargetPosition == intakeParkPosition && Math.abs(intakeLiftMotor.getSelectedSensorPosition() - intakeFullOutPosition) < 400) {
                        liftOut = -0.35;
                    }
                    break;
                case direct:
                    if(!auto) {
                        if(mOperatorInputManager.getPOV() == 0) { //set the intake forward
                            liftOut = 0.4;
                            targetState = true;
                        } else if(mOperatorInputManager.getPOV() == 180) {
                            liftOut = -0.4;
                            targetState = false;
                        } else {
                            if(targetState) {
                                liftOut = 0.08;
                            } else {
                                liftOut = -0.1;
                            }
                        }
                    }
                    break;
                default:
                    liftOut = 0;
                    break;
            }

            

            intakeLiftMotor.set(ControlMode.PercentOutput, liftOut);

            intakeRollerMotor.set(ControlMode.PercentOutput, rollerOut);

            try {Thread.sleep(10);} catch (InterruptedException ie) {} //deliberately only updates around 100hz
        }
        System.out.println("[Intake] left independent service");
    }
}
