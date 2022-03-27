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
    private TalonSRX secondWheel;

    private double shooterPower;
    private double shooterOut;
    private double lastCalibrationStageTS;
    private double calibrationPercentage;

    private boolean auto;
    private boolean autoConditionSatisfied;

    public enum ShooterControlMode {
        velocity,
        direct,
        calibration,
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
            case calibration:
                lastCalibrationStageTS = System.currentTimeMillis();
                calibrationPercentage = 1;
            default:
                break;
        }
        mShooterControlMode = mode;
    }

    public boolean getAutoConditionSatisfied() {
        return autoConditionSatisfied;
    }

    public void startShooterCalibration() {
        setShooterControlMode(ShooterControlMode.calibration);
    }

    private long integralTS;
    private double integralError;
    double shooterWheelSpeedToTicks(double centimetres){
        double ticksPerRevolution = 8192;
        double wheelSurcumfrenece = 30;
        double ticksPerCentimetter = ticksPerRevolution / wheelSurcumfrenece;
        return centimetres * ticksPerCentimetter;
    }
    private double biasedShooterPID(double target, double kP, double kI) {

        double error = target - shooterMotor.getSelectedSensorVelocity();

        integralError += ((System.nanoTime() - integralTS) * error) / 10000000000d;
        integralTS = System.nanoTime();

        return (error * kP) + (((target / 256) + 2.5) / 100);
    }

    private double shooterPID(double target, double kP, double kI) {
        double error = target - shooterMotor.getSelectedSensorVelocity();

        integralError += ((System.nanoTime() - integralTS) * error) / 10000000000d;
        integralTS = System.nanoTime();

        return (error * kP) + (integralError * kI);
    }

    private double secondWheelPID(double target, double kP, double kI) {
        double error = target - secondWheel.getSelectedSensorVelocity();

        integralError += ((System.nanoTime() - integralTS) * error) / 1000000000d;
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
        secondWheel = new TalonSRX(22); //re-used the backleft climb controller FIXME: re-program BL climb id to 12
        loaderMotor = new TalonSRX(11);
        shooterMotor.setSelectedSensorPosition(0);
        shooterMotor.setSensorPhase(true);
        shooterMotor.setInverted(false);
        secondWheel.setInverted(true);
        secondWheel.setSensorPhase(true);
        loaderMotor.setInverted(false);

        shooterMotor.configOpenloopRamp(0);

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
        SmartDashboard.putNumber("secondWheelSpeed", secondWheel.getSelectedSensorVelocity());
        SmartDashboard.putNumber("shooterCurrent", shooterMotor.getStatorCurrent());
        SmartDashboard.putNumber("secondWheelCurrent", secondWheel.getStatorCurrent());
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
            } else if(mOperatorInputManager.getEastButtonPressed()) {
                startShooterCalibration();;
            }
        }

        switch (mShooterControlMode) {
            case velocity:
                shooterOut = -biasedShooterPID(16000, 0.000421, 0.00085); // (target, P, I) 0.00000540 good at 16,000
                if(shooterOut > 0) {
                    shooterOut = 0;
                }
                shooterMotor.set(ControlMode.PercentOutput, shooterOut);
                //double secondWheelOut = secondWheelPID(21000, 0.00001, 0);
                double secondWheelOut = 0.2; //good at 0.6 for launchpad
                secondWheel.set(ControlMode.PercentOutput, secondWheelOut);
                //shooterMotor.set(ControlMode.PercentOutput, secondWheelOut);
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
            case calibration:
                shooterMotor.set(ControlMode.PercentOutput, -calibrationPercentage / 100);
                if(System.currentTimeMillis() - lastCalibrationStageTS > 500) {
                    double cumulative = 0;
                    for(int i = 0; i < 450; i++) {
                        cumulative += shooterMotor.getSelectedSensorVelocity();
                    }
                    double average = cumulative / 450;
                    System.out.println("Shooter cal: " + calibrationPercentage + ":" + average);
                    lastCalibrationStageTS = System.currentTimeMillis();
                    calibrationPercentage += 1;
                    if(calibrationPercentage > 75) {
                        setShooterControlMode(ShooterControlMode.none);
                    }
                }
                break;
            default:
                shooterMotor.set(ControlMode.PercentOutput, 0.0);
                loaderMotor.set(ControlMode.PercentOutput, 0);
                secondWheel.set(ControlMode.PercentOutput, 0);
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
