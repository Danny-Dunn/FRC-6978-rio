package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter extends Subsystem {
    private SubsystemManager mSubsystemManager;
    private LimelightController mLimelightController;
    
    private InputManager mDriverInputManager;
    private InputManager mOperatorInputManager;

    private TalonSRX shooterMotor; 
    private TalonSRX loaderMotor;
    private TalonSRX secondWheel;

    private double shooterTarget = 13000;
    private double secondWheelTarget = -1.0;
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

    private double secondWheelCalibration[][] = { //distance(cm), main wheel speed
        {440, -1.0},
        {385, -0.6},
        {336, -1.0},
        {250, -1.0},
        {170, -1.0},
        {115, -1.0},
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

        return (error * kP) + (getDesiredShooterVoltage(target) / shooterMotor.getBusVoltage());
    }

    private double getDesiredShooterVoltage(double setpoint) {
        return setpoint / shooterTicksPerVolt;
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

    public Shooter(InputManager inputManager, InputManager operatorInputManager, LimelightController limelightController) {
        mLimelightController = limelightController;
        mDriverInputManager = inputManager;
        mOperatorInputManager = operatorInputManager;
        SmartDashboard.putNumber("shooterTarget", shooterTarget);
        SmartDashboard.putNumber("secondWheelTarget", secondWheelTarget);
    }

    public boolean init() {
        shooterMotor = new TalonSRX(10);
        secondWheel = new TalonSRX(22); //re-used the backleft climb controller FIXME: re-program BL climb id to 12
        loaderMotor = new TalonSRX(11);
        shooterMotor.setSelectedSensorPosition(0);
        shooterMotor.setSensorPhase(true);
        shooterMotor.setInverted(true);
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
        SmartDashboard.putNumber("shooterCurrent", -shooterMotor.getStatorCurrent());
        SmartDashboard.putNumber("secondWheelCurrent", secondWheel.getStatorCurrent());
        SmartDashboard.putNumber("shooterOut", -shooterOut);
        SmartDashboard.putString("shooterControlMode", mShooterControlMode.toString());
        SmartDashboard.putBoolean("intakeLoaderEnabled", autoConditionSatisfied);

        //SmartDashboard.putNumber("shooterTarget", shooterTarget);

        if(takeConfigOptions) {
            shooterTarget = SmartDashboard.getNumber("shooterTarget", 13000);
            secondWheelTarget = SmartDashboard.getNumber("shooterWheelTarget", -1.0);
        }
    }

    public void run() {
        
        if(!auto) {
            if(mOperatorInputManager.getWestButtonPressed()) {
                setShooterControlMode(ShooterControlMode.velocity);
                if(mLimelightController.getShooterReady()) {
                    //double distance = mLimelightController.getDistanceFinal();
                    //shooterTarget = getCalibratedShooterSpeed(distance);
                    //System.out.println("Shooting at distance " + distance + " speed " + getCalibratedShooterSpeed(distance));
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
                shooterOut = biasedShooterPID(shooterTarget, 0.00028
                , 0.00085); // (target, P, I) 0.00000540 good at 16,000
                if(shooterOut < 0) {
                    shooterOut = 0;
                }
                shooterMotor.set(ControlMode.PercentOutput, shooterOut);
                //double secondWheelOut = secondWheelPID(21000, 0.00001, 0);
                double secondWheelOut = -0.6; //good at 0.6 for launchpad
                secondWheel.set(ControlMode.PercentOutput, secondWheelTarget);
                //shooterMotor.set(ControlMode.PercentOutput, secondWheelOut);

                if(Math.abs(shooterTarget - shooterMotor.getSelectedSensorVelocity()) < 150) {
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
                    double secondCumulative = 0;
                    for(int i = 0; i < 450; i++) {
                        cumulative += shooterMotor.getSelectedSensorVelocity();
                        secondCumulative += secondWheel.getSelectedSensorVelocity();
                    }

                    double average = cumulative / 450;
                    double secondAverage = secondCumulative / 450;
                    double ticksPerVolt = average / shooterMotor.getMotorOutputVoltage();
                    double secondTicksPerVolt = secondAverage / shooterMotor.getMotorOutputVoltage();
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
