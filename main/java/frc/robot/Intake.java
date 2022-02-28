package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Intake implements Runnable, ServiceableModule{
    private Thread mThread;

    InputManager mOperatorInputManager;

    TalonSRX intakeLiftMotor; //retracts ball pickup structure into robot
    TalonSRX intakeRollerMotor; //drives rollers to pick up cargo

    enum IntakeMode {
        user,
        none,
        position,
        rollers,
        all
    };

    IntakeMode mIntakeMode;

    boolean targetState = true; //determined by joystick buttons
    double intakeTargetPosition;
    double rollerPower;

    NetworkTable simTable;

    double intakeFullOutPosition = 750;
    double intakeParkPosition = -25;
    double intakeTicksPerRadian = 651.8986469;
    double intakeUpperPosition = -15;
    double intakeHorizontalBias = 0.14;

    private boolean autoConditionSatisfied;

    public boolean getAutoConditionSatisfied() {
        return autoConditionSatisfied;
    }

    public void setLift(double setpoint) {
        autoConditionSatisfied = false;
        intakeTargetPosition = setpoint;
        mIntakeMode = (mIntakeMode == IntakeMode.rollers)? IntakeMode.all : IntakeMode.position;
    }

    public void setRollers(double power) {
        autoConditionSatisfied = false;
        rollerPower = power;
        mIntakeMode = (mIntakeMode == IntakeMode.position)? IntakeMode.all : IntakeMode.rollers;
    }

    public void setUser() {
        mIntakeMode = IntakeMode.user;
        autoConditionSatisfied = false;
    }

    public Intake(InputManager inputManager) {
        mOperatorInputManager = inputManager;
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
        intakeLiftMotor.setSelectedSensorPosition(intakeUpperPosition);
        if(!RobotBase.isReal()) simTable = NetworkTableInstance.getDefault().getTable("simTable"); //simulation dummy outputs

        System.out.println("[Intake] finished initialisation");
        return true;
    }

    public boolean start() {
        return start(false);
    }

    public boolean start(boolean auto) {
        boolean ret;

        if(!auto) {
            ret = mOperatorInputManager.map();
            if(!ret) return ret;
        }

        exitFlag = false;
        if(auto) {
            mIntakeMode = IntakeMode.none;
        } else {
            mIntakeMode = IntakeMode.user;
        }

        mThread = new Thread(this, "Intake");
        mThread.start();

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
        SmartDashboard.putNumber("Intake Lift Radians", intakeLiftMotor.getSelectedSensorPosition() / intakeTicksPerRadian);
        SmartDashboard.putNumber("Intake Lift Power", intakeLiftMotor.getStatorCurrent() * intakeLiftMotor.getBusVoltage()); //calculate the total power draw in watts
        simOut("Intake Target Position", intakeTargetPosition);
    }

    long integralTS;
    double integralError;

    double intakeLiftPID(double target, double kP) {
        double error = target - intakeLiftMotor.getSelectedSensorPosition();

        double weightBias = Math.sin(intakeLiftMotor.getSelectedSensorPosition() / intakeTicksPerRadian) * intakeHorizontalBias;

        double movePower = (error * kP);
        movePower = (movePower > 0.10)? 0.10 : movePower;
        movePower = (movePower < -0.065)? -0.065 : movePower;
        SmartDashboard.putNumber("Move Power", movePower);
        SmartDashboard.putNumber("Intake Error", error);

        return weightBias + movePower;
    }

    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        exitFlag = false;
        intakeTargetPosition = 0;
        double deadZone = 0.1;
        double intakeFineAdjustSpeed = 0.5;
        long lastCalcTS = System.nanoTime();
        boolean intakeFineAdjustEnabled = true;
        boolean rollerFineAdjustEnabled = true;
        double rollerOutSpeed = -0.2;
        System.out.println("[Intake] entered independent service");
        while(!exitFlag) {
            double rollerOut = 0.0;
            switch(mIntakeMode) {
                case user:
                    double deltaT = (System.nanoTime() - lastCalcTS) / 1000000.0d;
                    lastCalcTS = System.nanoTime();

                    if(intakeFineAdjustEnabled) {
                        double y = -mOperatorInputManager.getLeftStickY();
                        y = (y < deadZone && y > -deadZone)? 0 : y;
                        if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                        y = y / (1 - deadZone); 

                        intakeTargetPosition += y * deltaT * intakeFineAdjustSpeed;
                    }

                    intakeTargetPosition = (intakeTargetPosition > intakeFullOutPosition)? intakeFullOutPosition : intakeTargetPosition;
                    intakeTargetPosition = (intakeTargetPosition < intakeParkPosition)? intakeParkPosition : intakeTargetPosition;            

                    if(mOperatorInputManager.getPOV() == 0) { //set the intake forward
                        intakeTargetPosition = intakeFullOutPosition;
                    } else if(mOperatorInputManager.getPOV() == 180) {
                        intakeTargetPosition = intakeParkPosition;
                    }
                    
                    

                    if(rollerFineAdjustEnabled) {
                        double y = mOperatorInputManager.getRightStickY();
                        y = (y < deadZone && y > -deadZone)? 0 : y;
                        if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                        y = y / (1 - deadZone); 

                        y = y*0.75;

                        simOut("Intake Roller Out", y);
                        rollerOut = y;
                    } else if(mOperatorInputManager.getSouthButton()) {
                        rollerOut = rollerOutSpeed;
                    } else if(mOperatorInputManager.getNorthButton()) {
                        rollerOut = rollerOutSpeed;
                    }
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
            }

            simOut("Intake Power Out", -intakeLiftPID(intakeTargetPosition, 0.0016));

            double liftOut = -intakeLiftPID(intakeTargetPosition, -0.0016);
            liftOut = (liftOut > 0.19)? 0.19 : liftOut;
            liftOut = (liftOut < -0.18)? -0.18 : liftOut;

            intakeLiftMotor.set(ControlMode.PercentOutput, liftOut);

            intakeRollerMotor.set(ControlMode.PercentOutput, rollerOut);

            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Intake] left independent service");
    }
}
