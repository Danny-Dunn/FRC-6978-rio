package frc.robot;
//Cross The Road Electronics(CTRE) libs must be installed
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;

//simOuts are for simulation purposes only, these can be removed in final robot code to improve efficiency

public class RealTimeDrive implements Runnable, ServiceableModule {
    Thread mThread;
    
    //motors
    TalonFX DL1Motor;
    TalonFX DL2Motor;
    TalonFX DR1Motor;
    TalonFX DR2Motor;
    public TalonSRX shooterMotor;
    TalonSRX loaderMotor;
    
    private InputManager mDriverInputManager;

    int calibrateButton;
    int autoButton;

    boolean firstCycle;

    NetworkTable simTable;

    Vector2d currentPosition;
    double oldLeftDrivePosition;
    double oldRightDrivePosition;
    double leftPosition;
    double rightPosition;
    double leftDrive;
    double rightDrive;
    double oldYaw;
    double absyaw;
    double realyaw;
    double ticksPerCentimetre = 1042.18; //new gearboxes
    //double ticksPerCentimetre = 385.47; //old gearboxes
    double delta; //generic delta variable used for BOTH position and angle
    AHRS navX;

    double angleOffset;

    public enum AutoMode {
        rotate,
        distance,
        curve,
        stop,
        user,
        none
    };

    AutoMode mAutoMode;

    enum DriveControlMode {
        direct,
        none
    };

    double targetAngle;
    double targetAngleOffset;
    double targetGyroRate;
    double targetDistance;
    double leftOffset;
    double rightOffset;
    boolean autoConditionSatisfied;

    //SmartDashboard sdb;

    double angleP;
    double angleP2;
    double distanceP;
    double angleI;
    double eIntegral;
    long angleTS;
    long autoTS;

    void setAutoMode(AutoMode dm) {
        mAutoMode = dm;
        autoConditionSatisfied = false;
        autoTS = System.nanoTime();
        angleTS = System.nanoTime();
        double absDelta;
        switch(dm) {
            case rotate:
                targetAngleOffset = realyaw;
                //targetAngle = targetAngle - targetAngleOffset;
                absDelta = targetAngle - absyaw;
                if(absDelta > 180) {
                    absDelta = absDelta - 360;
                } else if(absDelta < -180) {
                    absDelta = absDelta + 360;
                }
                targetAngle = absDelta;
                eIntegral = 0;
                System.out.println("Turning " + targetAngle);
                SmartDashboard.putNumber("targetOffset", targetAngleOffset);
                SmartDashboard.putNumber("absdelta", absDelta);
                break;
            case distance:
                eIntegral = 0;
                leftOffset = leftPosition;
                rightOffset = rightPosition;
            case curve:
                leftOffset = leftPosition;
                rightOffset = rightPosition;
                targetAngleOffset = realyaw;
                absDelta = targetAngle - absyaw;
                if(absDelta > 180) {
                    absDelta = absDelta - 360;
                } else if(absDelta < -180) {
                    absDelta = absDelta + 360;
                }
                targetAngle = absDelta;
            default:
                break;
        }
    }

    Vector2d calcGraphTransition(Vector2d lastPosition, double distance, double yaw) {
		double radius = (distance) / ticksPerCentimetre;
		double radians = Math.toRadians(yaw);
		//calc the new point
        lastPosition.y = lastPosition.y + (radius * Math.sin(radians));
        lastPosition.x = lastPosition.x + (radius * Math.cos(radians));
        return lastPosition;
	}

    void advanceTracking() {
        leftPosition = DL1Motor.getSelectedSensorPosition();
        rightPosition = -DR1Motor.getSelectedSensorPosition();
        
        realyaw = navX.getAngle() - angleOffset;
        absyaw = Math.abs(realyaw % 360);

        currentPosition = calcGraphTransition(currentPosition, ((leftPosition - oldLeftDrivePosition) + (rightPosition - oldRightDrivePosition)) / 2, (absyaw + oldYaw) / 2);
        
        oldYaw = absyaw;
        oldLeftDrivePosition = leftPosition;
        oldRightDrivePosition = rightPosition;
    }

    boolean calibrateTracking(boolean calibrateNavx) {
        currentPosition.x = 0;
        currentPosition.y = 0;
        DL1Motor.setSelectedSensorPosition(0);
        DL2Motor.setSelectedSensorPosition(0);
        DR1Motor.setSelectedSensorPosition(0);
        DR2Motor.setSelectedSensorPosition(0);

        if(calibrateNavx) {
            navX.calibrate();
            long waitTS = System.currentTimeMillis();
            while((!navX.isConnected()) || navX.isCalibrating())
            {
                if(System.currentTimeMillis() - waitTS > 1000) return false;
                try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
            }
            System.out.println("[RTDrive] navX finished cal after " + waitTS + "ms");
        }

        double cumulative = 0;
        for(int i = 0; i < 450; i++) {
            cumulative += navX.getAngle();
        }
        angleOffset = cumulative / 450;
        return true;
    }

    void setDriveMotors(double left, double right) {
        DL1Motor.set(ControlMode.PercentOutput, left);
        DL2Motor.set(ControlMode.PercentOutput, left);
        DR1Motor.set(ControlMode.PercentOutput, -right);
        DR2Motor.set(ControlMode.PercentOutput, -right);
    }

    public RealTimeDrive(InputManager inputManager, AHRS navX) {
        //meta stuff
        this.mDriverInputManager = inputManager;
        this.navX = navX;
        SmartDashboard.putNumber("angleP", 0.0418);
        SmartDashboard.putNumber("angleP2", 0.0104);
        SmartDashboard.putNumber("angleI", 0.0085);
        SmartDashboard.putNumber("distanceP", 0.046);
        SmartDashboard.putBoolean("AutoConditionSatisfied", autoConditionSatisfied);
    }
    //output functions for simulation
    public void simOut(String tag, Double value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setDouble(value);
        } else {
            SmartDashboard.putNumber(tag, value);
        }
    }
    public void simOut(String tag, Boolean value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setBoolean(value);
        } else {
            SmartDashboard.putBoolean(tag, value);
        }
    }

    public double getShooterVelocity() {
        return shooterMotor.getSelectedSensorVelocity();
    }

    public boolean init() {
        //setup motors
        DL1Motor = new TalonFX(1);
        DL2Motor = new TalonFX(2);
        DR1Motor = new TalonFX(3);
        DR2Motor = new TalonFX(4);

        if(!RobotBase.isReal()) simTable = NetworkTableInstance.getDefault().getTable("simTable"); //simulation dummy outputs

        currentPosition = new Vector2d(0, 0);

        if(!calibrateTracking(true)) return false;
        System.out.println("[RTDrive] Calibrated tracking with angle offset " + angleOffset);
        
        mAutoMode = AutoMode.none;

        mDriverInputManager.map();

        System.out.println("[RTDrive] finished initialisation");
        return true; //everything went fine??
    }

    public boolean start() {
        boolean ret;

        currentPosition = new Vector2d(0, 0);
        ret = calibrateTracking(false);
        System.out.println("[RTDrive] Calibrated tracking with angle offset " + angleOffset);


        ret = mDriverInputManager.map();
        if(!ret) return ret;
        
        exitFlag = false;

        mThread = new Thread(this, "RTDrive");
        mThread.start();

        mAutoMode = AutoMode.user;

        return true;
    }

    public boolean stop() {
        if(mThread == null) return true;

        setDriveMotors(0, 0);
        SmartDashboard.putBoolean("RTDrive OK", false);

        exitFlag = true;
        try {Thread.sleep(20);} catch (Exception e) {}

        if(mThread.isAlive()) {
            mThread.interrupt();
            try {Thread.sleep(20);} catch (Exception e) {}
            if(mThread.isAlive()) {
                return false;
            }
        }

        mThread = null;

        mAutoMode = AutoMode.none;

        return true;
    }

    //standby function should be called every time the module should do things like report telemetry
    public void standby(boolean takeConfigOptions) {
        simOut("leftDrive", leftDrive);
        simOut("rightDrive", rightDrive);

        SmartDashboard.putNumber("DL1Temp", DL1Motor.getTemperature());
        SmartDashboard.putNumber("DL2Temp", DL2Motor.getTemperature());
        SmartDashboard.putNumber("DR1Temp", DR1Motor.getTemperature());
        SmartDashboard.putNumber("DR2Temp", DR2Motor.getTemperature());

        SmartDashboard.putNumber("DL1Current", DL1Motor.getStatorCurrent());
        SmartDashboard.putNumber("DL2Current", DL2Motor.getStatorCurrent());
        SmartDashboard.putNumber("DR1Current", DR1Motor.getStatorCurrent());
        SmartDashboard.putNumber("DR2Current", DR2Motor.getStatorCurrent());

        SmartDashboard.putNumber("targetDelta", delta);
        SmartDashboard.putBoolean("AutoConditionSatisfied", autoConditionSatisfied);
        SmartDashboard.putNumber("gyroRate", navX.getRate());

        SmartDashboard.putNumber("SpeedL", DL1Motor.getSelectedSensorVelocity());
        SmartDashboard.putNumber("SpeedR", -DR1Motor.getSelectedSensorVelocity());

        SmartDashboard.putNumber("trackX", currentPosition.x);
        SmartDashboard.putNumber("trackY", currentPosition.y);
        SmartDashboard.putNumber("DistanceL", leftPosition);
        SmartDashboard.putNumber("DistanceR", rightPosition);
        SmartDashboard.putNumber("Yaw", realyaw);
        SmartDashboard.putNumber("absYaw", absyaw);

        if(mAutoMode != null)
        SmartDashboard.putString("autoGuidanceMode", mAutoMode.toString());

        if(takeConfigOptions) {
            angleP = SmartDashboard.getNumber("angleP", 0.0015);
            angleP2 = SmartDashboard.getNumber("angleP2", 0.0104);
            angleI = SmartDashboard.getNumber("angleI", 0.00084);
            distanceP = SmartDashboard.getNumber("distanceP", 0.0045);
        }
    }

    void forcefulDisconnect(String reason) {
        System.out.println("[RTDrive] CRITICAL!! " + reason);
        System.out.println("[RTDrive] disconnected");
        stop();
        return;
    }

    public boolean exitFlag;
    public void run() { //might remove
        System.out.println("[RTDrive] entered independent service");
        
        firstCycle = true;

        while (!exitFlag) {
            long start = System.nanoTime();
            
            advanceTracking();

            if(mDriverInputManager.getRightSystemButton()) {
                calibrateTracking(true);
            }

            delta = 0.0;
            double maxTurn = 0.45;
            double deltaT;

            double deadZone = 0.2;
            double fullSpeed = 0.75;

            double x = 0.0;
            double y = 0.0;

            switch(mAutoMode) {
                case rotate:
                    delta = targetAngle - (realyaw - targetAngleOffset);

                    eIntegral += ((System.nanoTime() - angleTS) * delta) / 10000000000d;
                    angleTS = System.nanoTime();
                    SmartDashboard.putNumber("eIntegral", eIntegral);

                    x = (delta * angleP) + (eIntegral * angleI);

                    x = (x > maxTurn)? maxTurn: x;
                    x = (x < -maxTurn)? -maxTurn: x;
                    //
                    y = 0;

                    autoConditionSatisfied = (Math.abs(delta) < 2.0) && (Math.abs(navX.getRate()) < 0.02); //auto is satisfied if almost still
                    break;

                case distance:
                    delta = targetDistance - ((((leftPosition - leftOffset)+(rightPosition - rightOffset))/2) / ticksPerCentimetre);
                    x = 0;
                    y = delta * distanceP;
                    autoConditionSatisfied = (Math.abs(delta) < 2.0);
                    break;

                case curve:
                    double angleRateP = 0.5;
                    double angleRateI = 0.01;

                    //double progress = ((((leftPosition - leftOffset)+(rightPosition - rightOffset))/2) / ticksPerCentimetre) / targetDistance; //0 to 1 of the distance we have traveled
                    
                    double distanceDelta = targetDistance - ((((leftPosition - leftOffset)+(rightPosition - rightOffset))/2) / ticksPerCentimetre);
                    y = distanceDelta * distanceP;

                    double rateDelta = targetGyroRate - (navX.getRate());

                    deltaT = (System.nanoTime() - angleTS) / 100000000;
                    angleTS = System.nanoTime();

                    eIntegral += rateDelta * deltaT;

                    x = (rateDelta * angleRateP) + (eIntegral * angleRateI);

                    x = (x > maxTurn)? maxTurn: x;
                    x = (x < -maxTurn)? -maxTurn: x;
                    break;

                case stop:
                    x = 0;
                    y = 0;
                    break;

                case user: //USER control
                    double Lt = mDriverInputManager.getLeftTrigger();
                    double Rt = mDriverInputManager.getRightTrigger();
                    
                    
                    if(Lt < 0 || Rt < 0) {
                        forcefulDisconnect("invalid trigger inputs " + Lt + " " + Rt);
                        return;
                    }
                    
                    y = Rt - Lt;
                    x = mDriverInputManager.getLeftStickX();
                    
                    //deadzone calclations
                    x = (x < deadZone && x > -deadZone)? 0 : x;
                    if(x != 0.0) x = (x > 0.0)? x - deadZone : x + deadZone; //eliminate jump behaviour
                    x = x / (1 - deadZone);
                    simOut("xval", x);

                    double aparam = 0.6;

                    x = (aparam * (x * x * x)) + ((1-aparam) * x);
                    
                    //y = (y < deadZone && y > -deadZone)? 0 : y;
                    //if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                    //y = y / (1 - deadZone);
                    simOut("yval", y);
                    break;
                default:
                    break;
            }

            leftDrive = y + x;
            rightDrive = y - x;

            //speed scaling
            leftDrive = leftDrive * fullSpeed; 
            rightDrive = rightDrive * fullSpeed;

            //speed hard cap
            leftDrive = (leftDrive > fullSpeed)? fullSpeed : leftDrive;
            rightDrive = (rightDrive > fullSpeed)? fullSpeed : rightDrive;
            leftDrive = (leftDrive < -fullSpeed)? -fullSpeed : leftDrive;
            rightDrive = (rightDrive < -fullSpeed)? -fullSpeed : rightDrive;

            if(firstCycle) {
                if(leftDrive != 0 || rightDrive != 0) {
                    forcefulDisconnect("drive state not zeroed on startup");
                    return;
                }
                SmartDashboard.putBoolean("RTDrive OK", true);
                firstCycle = false;
            }

            setDriveMotors(leftDrive, rightDrive);
            
            long elapsedTime = System.nanoTime() - start;
            if(elapsedTime > 2000000) {
                //System.out.println("[RTDrive] Motion processing took longer than 2ms! Took " + elapsedTime + "uS");
            }
        }
        SmartDashboard.putBoolean("RTDrive OK", false);
        System.out.println("[RTDrive] left independent service");
    }
}