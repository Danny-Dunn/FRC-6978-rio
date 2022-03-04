package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter implements Runnable, ServiceableModule {
    private Thread mThread;
    
    private InputManager mDriverInputManager;

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
                if(mShooterControlMode == ShooterControlMode.velocity) {
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
    private double shooterPID(double target, double kP, double kI) {
        double error = target - shooterMotor.getSelectedSensorVelocity();

        integralError += ((System.nanoTime() - integralTS) * error) / 10000000000d;
        integralTS = System.nanoTime();

        return (error * kP) + (integralError * kI);
    }

    public Shooter(InputManager inputManager) {
        mDriverInputManager = inputManager;
        SmartDashboard.putNumber("Shooter Power", 0.6);
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
        return true;
    }

    public boolean start() {
        return start(false);
    }

    public boolean start(boolean auto) {
        boolean ret;

        if(!auto) {
            ret = mDriverInputManager.map();
            if(!ret) return ret;
        }
        this.auto = auto;

        exitFlag = false;

        mThread = new Thread(this, "Shooter");
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
        SmartDashboard.putNumber("shooterSpeed", shooterMotor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Shooter Current", shooterMotor.getStatorCurrent());
        SmartDashboard.putNumber("shooterOut", shooterOut);

        SmartDashboard.putString("shooterControlMode", mShooterControlMode.toString());

        SmartDashboard.putBoolean("intakeLoaderEnabled", autoConditionSatisfied);

        if(takeConfigOptions) {
            shooterPower = SmartDashboard.getNumber("Shooter Power", 0.6);
        }
    }
    
    public boolean exitFlag; //this flag is set true when the loop is to be exited
    public void run() {
        
        System.out.println("[Shooter] entered independent service");
        while(!exitFlag) {
            if(!auto) {
                if(mDriverInputManager.getSouthButtonPressed()) {
                    setShooterControlMode(ShooterControlMode.velocity);
                } else if(mDriverInputManager.getSouthButtonReleased()) {
                    setShooterControlMode(ShooterControlMode.none);
                }
            }

            switch (mShooterControlMode) {
                case velocity:
                    shooterMotor.set(ControlMode.PercentOutput, -shooterPID(19000, 0.000511, 0.00000068));
                    shooterOut = -shooterPID(19000, 0.000511, 0.00000068);
                    if(19000 - shooterMotor.getSelectedSensorVelocity() < 500 || mDriverInputManager.getWestButton()) {
                        loaderMotor.set(ControlMode.PercentOutput, 0.2);
                        autoConditionSatisfied = true;
                    } else {
                        loaderMotor.set(ControlMode.PercentOutput, 0);
                        autoConditionSatisfied = false;
                    }
                    break;
                case direct:
                    shooterMotor.set(ControlMode.PercentOutput, shooterPower);
                    if(mDriverInputManager.getWestButton()) {
                        loaderMotor.set(ControlMode.PercentOutput, 0.2);
                    } else {
                        loaderMotor.set(ControlMode.PercentOutput, 0);
                    }
                default:
                    shooterMotor.set(ControlMode.PercentOutput, 0.0);
                    loaderMotor.set(ControlMode.PercentOutput, 0);
                    autoConditionSatisfied = false;
                    break;
            }

            try {Thread.sleep(5);} catch (InterruptedException ie) {} //deliberately only updates around 200hz
        }
        System.out.println("[Shooter] left independent service");
    }
}
