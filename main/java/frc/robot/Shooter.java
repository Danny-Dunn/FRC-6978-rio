package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.controllers.CalibratedVelocityController;
import frc.robot.driver.TalonSRXDriver;

public class Shooter extends Subsystem {
    private LimelightController mLimelightController;
    
    private InputManager mDriverInputManager;
    private InputManager mOperatorInputManager;

    private MotorDriver mShooterDriver; 
    private CalibratedVelocityController mShooterController;

    private MotorDriver mSecondWheelDriver;
    private CalibratedVelocityController mSecondWheelController;

    private TalonSRX loaderMotor;
    

    private double shooterTarget = 13000;
    private double secondWheelTarget = 20000;
    private double shooterOut;
    private double lastCalibrationStageTS;
    private double calibrationPercentage;

    private boolean auto;
    private boolean autoConditionSatisfied;

    private long stabilityTS;
    private boolean shooterWheelStable;

    private double shooterTicksPerVolt = 2050;

    private double shooterCalibration[][] = { //distance(cm), main wheel speed
        {440, 15800},
        {385, 14500},
        {336, 13500},
        {250, 10500},
        {170, 8900},
        {115, 7700},
    };

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
                    mShooterController.prepare();
                    mSecondWheelController.prepare();
                }
                break;
            case calibration:
                lastCalibrationStageTS = System.currentTimeMillis();
                calibrationPercentage = 1;
                break;
            default:
                mShooterDriver.disable();
                mSecondWheelDriver.disable();
                break;
        }
        mShooterControlMode = mode;
    }

    public void setShooterSpeed(double shooterSpeed) {
        shooterTarget = shooterSpeed;
    }

    public void runLoader(double runSpeed){
        loaderMotor.set(ControlMode.PercentOutput, runSpeed); 
    }

    public boolean getAutoConditionSatisfied() {
        return autoConditionSatisfied;
    }

    public void startShooterCalibration() {
        setShooterControlMode(ShooterControlMode.calibration);
    }

    private double getCalibratedShooterSpeed(double distance) {
        double closestLower = 0, closestUpper = 1500;
        int closestLowerIndex = 0, closestUpperIndex = 0;
        //find the closest calibration points above and below the current distance
        for(int i=0; i<shooterCalibration.length; i++) {
            if(shooterCalibration[i][0] > distance && shooterCalibration[i][0] < closestUpper) {
                closestUpper = shooterCalibration[i][0]; 
                closestUpperIndex = i;
            }
            if(shooterCalibration[i][0] < distance && shooterCalibration[i][0] > closestLower) {
                closestLower = shooterCalibration[i][0]; 
                closestLowerIndex = i;
            }
        }

        double tween = (distance - closestLower) / (closestUpper - closestLower);

        //System.out.println("Closest lower, upper: " + closestLower + ", " + closestUpper);

        //System.out.println("Tween: " + tween);


        return shooterCalibration[closestLowerIndex][1] + (tween * (shooterCalibration[closestUpperIndex][1] - shooterCalibration[closestLowerIndex][1]));
    }

    public Shooter(InputManager inputManager, InputManager operatorInputManager, LimelightController limelightController) {
        mLimelightController = limelightController;
        mDriverInputManager = inputManager;
        mOperatorInputManager = operatorInputManager;
        SmartDashboard.putNumber("shooterTarget", shooterTarget);
        SmartDashboard.putNumber("secondWheelTarget", secondWheelTarget);
    }

    public boolean init() {
        mShooterDriver = new TalonSRXDriver(10);
        
        mShooterDriver.setRelativePosition(0);
        mShooterDriver.setEncoderInversion(true);
        mShooterDriver.setDriverInversion(true);
        
        mShooterController = new CalibratedVelocityController(mShooterDriver, shooterTicksPerVolt);
        mShooterController.setPIDConstants(0.00028, 0, 0);
        
        mSecondWheelDriver = new TalonSRXDriver(22); //re-used the backleft climb controller FIXME: re-program BL climb id to 12
        
        mSecondWheelDriver.setDriverInversion(true);
        mSecondWheelDriver.setEncoderInversion(true);

        mSecondWheelController = new CalibratedVelocityController(mSecondWheelDriver, 2050); //FIXME: second wheel calibration
        mSecondWheelController.setPIDConstants(0, 0, 0); //FIXME: determine PID constants for second wheel

        loaderMotor = new TalonSRX(11);
        
        
        loaderMotor.setInverted(false);

        setShooterControlMode(ShooterControlMode.none);

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
        SmartDashboard.putNumber("shooterSpeed", mShooterDriver.getVelocity());
        SmartDashboard.putNumber("secondWheelSpeed", mSecondWheelDriver.getVelocity());
        SmartDashboard.putNumber("shooterCurrent", mShooterDriver.getOutputCurrent());
        SmartDashboard.putNumber("secondWheelCurrent", mSecondWheelDriver.getOutputCurrent());
        SmartDashboard.putNumber("shooterOut", -shooterOut);
        SmartDashboard.putString("shooterControlMode", mShooterControlMode.toString());
        SmartDashboard.putBoolean("intakeLoaderEnabled", autoConditionSatisfied);

        //SmartDashboard.putNumber("shooterTarget", shooterTarget);

        if(takeConfigOptions) {
            shooterTarget = SmartDashboard.getNumber("shooterTarget", 13000);
            secondWheelTarget = SmartDashboard.getNumber("secondWheelTarget", 20000);
        }
    }

    public void run() {
        
        if(!auto) {
            if(mOperatorInputManager.getWestButtonPressed()) {
                setShooterControlMode(ShooterControlMode.velocity);
                if(mLimelightController.getShooterReady()) {
                    double distance = mLimelightController.getDistanceFinal();
                    shooterTarget = getCalibratedShooterSpeed(distance);
                    System.out.println("Shooting at distance " + distance + " speed " + getCalibratedShooterSpeed(distance));
                } else {
                    //shooterTarget = 9100;
                    System.out.println("[Shooter] Limelight not ready, setting default 170cm shot");
                }
            } else if(mOperatorInputManager.getWestButtonReleased()) {
                setShooterControlMode(ShooterControlMode.none);
            } else if(mOperatorInputManager.getEastButtonPressed()) {
                startShooterCalibration();;
            }
        }

        switch (mShooterControlMode) {
            case velocity:
                mShooterController.setVelocityDemand(shooterTarget);
                mShooterController.refresh();

                mSecondWheelController.setVelocityDemand(secondWheelTarget);
                mSecondWheelController.refresh();

                if(Math.abs(shooterTarget - mShooterDriver.getVelocity()) < 150) {
                    if(!shooterWheelStable) {
                        shooterWheelStable = true;
                        stabilityTS = System.currentTimeMillis();
                    }

                } else {
                    shooterWheelStable = false;
                }

                if((System.currentTimeMillis() - stabilityTS > 100 && shooterWheelStable) || mDriverInputManager.getEastButton()) { //if the wheel has been stable for 100ms
                    loaderMotor.set(ControlMode.PercentOutput, 1);
                    autoConditionSatisfied = true;
                } else {
                    loaderMotor.set(ControlMode.PercentOutput, 0);
                    autoConditionSatisfied = false;
                }
                break;
            case direct:
                mShooterDriver.set(-0.7);
                /*if(mDriverInputManager.getWestButton()) {
                    loaderMotor.set(ControlMode.PercentOutput, 0.08);
                } else {
                    loaderMotor.set(ControlMode.PercentOutput, 0);
                }*/
                break;
            case calibration:
                mShooterDriver.set(calibrationPercentage / 100);
                if(System.currentTimeMillis() - lastCalibrationStageTS > 500) {
                    double cumulative = 0;
                    double secondCumulative = 0;
                    for(int i = 0; i < 450; i++) {
                        cumulative += mShooterDriver.getVelocity();
                        secondCumulative += mSecondWheelDriver.getVelocity();
                    }

                    double average = cumulative / 450;
                    double secondAverage = secondCumulative / 450;
                    double ticksPerVolt = average / mShooterDriver.getOutputVoltage();
                    double secondTicksPerVolt = secondAverage / mSecondWheelDriver.getOutputVoltage();
                    System.out.println("Shooter cal: " + calibrationPercentage + ":" + average + ":" + ticksPerVolt);
                    System.out.println("Second cal: " + calibrationPercentage + ":" + secondAverage + ":" + secondTicksPerVolt);
                    lastCalibrationStageTS = System.currentTimeMillis();
                    calibrationPercentage += 1;
                    if(calibrationPercentage > 50) {
                        setShooterControlMode(ShooterControlMode.none);
                    }
                }
                break;
            default:
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
