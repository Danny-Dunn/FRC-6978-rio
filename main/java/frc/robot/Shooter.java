package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private SubsystemManager mSubsystemManager;
    
    private InputManager mDriverInputManager;
    private InputManager mOperatorInputManager;

    private TalonSRX shooterMotor; 
    private TalonSRX loaderMotor;

    private double shooterPower;
    private double shooterOut;

    private boolean auto;
    private boolean autoConditionSatisfied;

    public enum ShooterControlMode {
        velocity,
        direct,
        none
    };

    private ShooterControlMode mShooterControlMode;

    public void setShooterControlMode(ShooterControlMode mode) {
        System.out.println("[Shooter] set control mode to " + mode.toString());
        switch (mode) {
            case velocity:
                if(mShooterControlMode != ShooterControlMode.velocity) {
                    integralError = 0;
                    integralTS = System.nanoTime();
                }
                break;
            default:
                break;
        }
        mShooterControlMode = mode;
    }

    public boolean getAutoConditionSatisfied() {
        return autoConditionSatisfied;
    }

    private long integralTS;
    private double integralError;
    private double biasedShooterPID(double target, double kP, double kI) {
        double error = target - shooterMotor.getSelectedSensorVelocity();

        integralError += ((System.nanoTime() - integralTS) * error) / 10000000000d;
        integralTS = System.nanoTime();

        return (error * kP) + /*(integralError * kI)+*/ 0.7;
    }

    private double shooterPID(double target, double kP, double kI) {
        double error = target - shooterMotor.getSelectedSensorVelocity();

        integralError += ((System.nanoTime() - integralTS) * error) / 10000000000d;
        integralTS = System.nanoTime();

        return (error * kP) + (integralError * kI);
    }

    public Shooter(InputManager inputManager, InputManager operatorInputManager) {
        mDriverInputManager = inputManager;
        mOperatorInputManager = operatorInputManager;
        SmartDashboard.putNumber("Shooter Power", 0.7);
    }

    public boolean init() {
        shooterMotor = new TalonSRX(10);
        loaderMotor = new TalonSRX(11);
        shooterMotor.setSelectedSensorPosition(0);
        shooterMotor.setSensorPhase(true);
        shooterMotor.setInverted(false);
        loaderMotor.setInverted(true);

        mShooterControlMode = ShooterControlMode.none;

        System.out.println("[Shooter] finished initialisation");

        //manager = subsystemManager;
        return true;
    }

    public boolean setup() {
        boolean ret;
        auto = false;
        if(!auto) {
            ret = mOperatorInputManager.map();
            if(!ret) {
                manager.setFailure("Controller mapping failed");
                return ret;
            }
        }
        setShooterControlMode(ShooterControlMode.none);
        return true;
    }

    public void standby(boolean takeConfigOptions) {
        SmartDashboard.putNumber("shooterSpeed", shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooterCurrent", shooterMotor.getStatorCurrent());
        SmartDashboard.putNumber("shooterOut", shooterOut);

        SmartDashboard.putString("shooterControlMode", mShooterControlMode.toString());

        SmartDashboard.putBoolean("intakeLoaderEnabled", autoConditionSatisfied);

        if(takeConfigOptions) {
            shooterPower = SmartDashboard.getNumber("Shooter Power", 0.6);
        }
    }
    
    public void run() {
        
        if(!auto) {
            if(mOperatorInputManager.getWestButtonPressed()) {
                setShooterControlMode(ShooterControlMode.velocity);
            } else if(mOperatorInputManager.getWestButtonReleased()) {
                setShooterControlMode(ShooterControlMode.none);
            }
        }

        switch (mShooterControlMode) {
            case velocity:
                shooterOut = -shooterPID(16000, 0.000421, 0.00085); // (target, P, I) 0.00000540
                shooterMotor.set(ControlMode.PercentOutput, shooterOut);
                
                if(16000 - shooterMotor.getSelectedSensorVelocity() < 250 || mDriverInputManager.getWestButton()) {
                    loaderMotor.set(ControlMode.PercentOutput, 0.35);
                    autoConditionSatisfied = true;
                } else {
                    loaderMotor.set(ControlMode.PercentOutput, 0);
                    autoConditionSatisfied = false;
                }
                break;
            case direct:
                shooterMotor.set(ControlMode.PercentOutput, -0.7);
                /*if(mDriverInputManager.getWestButton()) {
                    loaderMotor.set(ControlMode.PercentOutput, 0.08);
                } else {
                    loaderMotor.set(ControlMode.PercentOutput, 0);
                }*/
                break;
            default:
                shooterMotor.set(ControlMode.PercentOutput, 0.0);
                loaderMotor.set(ControlMode.PercentOutput, 0);
                autoConditionSatisfied = false;
                break;
        }

        if(!auto) {
            if(mOperatorInputManager.getRightSystemButton()) {
                loaderMotor.set(ControlMode.PercentOutput, 0.17);
            }
            if(mOperatorInputManager.getLeftSystemButton()) {
                loaderMotor.set(ControlMode.PercentOutput, -0.12);
            }
        }

        try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
    }
}
