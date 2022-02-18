package frc.robot;
//Cross The Road Electronics(CTRE) libs must be installed
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;

//simOuts are for simulation purposes only, these can be removed in final robot code to improve efficiency

public class RealTimeDrive implements Runnable, ServiceableModule {
    //motors
    TalonFX DL1Motor;
    TalonFX DL2Motor;
    TalonFX DR1Motor;
    TalonFX DR2Motor;
    public TalonSRX shooterMotor;
    TalonSRX loaderMotor;
    
    Joystick driverStick;

    int calibrateButton;
    int autoButton;

    boolean firstCycle;

    private enum ControllerType {
        PS5,
        standard,
    };

    private ControllerType controllerType;

    public double aimInput;
    public double aimInputy;
    public double shooterInput;

    boolean alignEnabled;
    public boolean alignDCOK;
    public boolean shooterEnabled;

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

    public enum DriveMode {
        rotate,
        distance,
        curve,
        stop
    };

    DriveMode mode;
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

    void setDriveMode(DriveMode dm) {
        mode = dm;
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
        //double realrealyawImeanitthistime = navX.getYaw();
        absyaw = Math.abs(realyaw % 360);
        //double roll = navX.getRoll();
        //double pitch = navX.getPitch();
        currentPosition = calcGraphTransition(currentPosition, ((leftPosition - oldLeftDrivePosition) + (rightPosition - oldRightDrivePosition)) / 2, (absyaw + oldYaw) / 2);
        oldYaw = absyaw;
        oldLeftDrivePosition = leftPosition;
        oldRightDrivePosition = rightPosition;
        //TODO: Maybe remove these as they may cost performance
        SmartDashboard.putNumber("trackX", currentPosition.x);
        SmartDashboard.putNumber("trackY", currentPosition.y);
        SmartDashboard.putNumber("DistanceL", leftPosition);
        SmartDashboard.putNumber("DistanceR", rightPosition);
        //SmartDashboard.putNumber("RealRealYaw", realrealyawImeanitthistime);
        SmartDashboard.putNumber("Yaw", realyaw);
        SmartDashboard.putNumber("absYaw", absyaw);
        //SmartDashboard.putNumber("Pitch", pitch);
    }

    void calibrateTracking() {
        //navX.calibrate();
        currentPosition.x = 0;
        currentPosition.y = 0;
        DL1Motor.setSelectedSensorPosition(0);
        DL2Motor.setSelectedSensorPosition(0);
        DR1Motor.setSelectedSensorPosition(0);
        DR2Motor.setSelectedSensorPosition(0);
        navX.calibrate();
        double cumulative = 0;
        for(int i = 0; i < 450; i++) {
            cumulative += navX.getAngle();
        }
        angleOffset = cumulative / 450;
        SmartDashboard.putNumber("offset", angleOffset);
    }

    public RealTimeDrive(Joystick driveStick, AHRS navX) {
        //meta stuff
        this.driverStick = driveStick;
        this.navX = navX;
        this.aimInput = 0;
        SmartDashboard.putNumber("angleP", 0.0158);
        SmartDashboard.putNumber("angleP2", 0.0104);
        SmartDashboard.putNumber("angleI", 0.000007);
        SmartDashboard.putNumber("distanceP", 0.016);
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
        //navX = new AHRS(SPI.Port.kMXP);
        while((!navX.isConnected()) || navX.isCalibrating())
        {
            try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
        calibrateTracking();
        System.out.println("[RTDrive] Calibrated tracking with angle offset " + angleOffset);
        mode = DriveMode.stop;

        if(driverStick.getName().equals("Wireless Controller")) {
            System.out.println("[RTDrive] detected PS5 controller, switching mapping");
            controllerType = ControllerType.PS5;
            calibrateButton = 10;
            autoButton = 9;
        } else {
            controllerType = ControllerType.standard;
            System.out.println("[RTDrive] using default mapping");
            calibrateButton = 8;
            autoButton = 7;
        }

        System.out.println("[RTDrive] finished initialisation");

        return true; //everything went fine??
    }
    //standby function should be called every time the module should do things like report telemetry
    public void standby(boolean takeConfigOptions) {
        simOut("leftDrive", leftDrive);
        simOut("rightDrive", rightDrive);

        SmartDashboard.putNumber("DL1 Temp", DL1Motor.getTemperature());
        SmartDashboard.putNumber("DL2 Temp", DL2Motor.getTemperature());
        SmartDashboard.putNumber("DR1 Temp", DR1Motor.getTemperature());
        SmartDashboard.putNumber("DR2 Temp", DR2Motor.getTemperature());

        SmartDashboard.putNumber("DL1 Current", DL1Motor.getStatorCurrent());
        SmartDashboard.putNumber("DL2 Current", DL2Motor.getStatorCurrent());
        SmartDashboard.putNumber("DR1 Current", DR1Motor.getStatorCurrent());
        SmartDashboard.putNumber("DR2 Current", DR2Motor.getStatorCurrent());

        SmartDashboard.putNumber("targetDelta", delta);
        SmartDashboard.putBoolean("AutoConditionSatisfied", autoConditionSatisfied);
        SmartDashboard.putNumber("gyroRate", navX.getRate());

        if(takeConfigOptions) {
            angleP = SmartDashboard.getNumber("angleP", 0.0015);
            angleP2 = SmartDashboard.getNumber("angleP2", 0.0104);
            angleI = SmartDashboard.getNumber("angleI", 0.00084);
            distanceP = SmartDashboard.getNumber("distanceP", 0.002);
        }
    }

    void forcefulDisconnect(String reason) {
        System.out.println("[RTDrive] CRITICAL!! " + reason);
        System.out.println("[RTDrive] disconnected");
        DL1Motor.set(ControlMode.PercentOutput, 0);
        DL2Motor.set(ControlMode.PercentOutput, 0);
        DR1Motor.set(ControlMode.PercentOutput, 0);
        DR2Motor.set(ControlMode.PercentOutput, 0);
        SmartDashboard.putBoolean("RTDrive OK", false);
        return;
    }

    public boolean exitFlag;
    public void run() { //might remove
        System.out.println("[RTDrive] entered independent service");
        exitFlag = false; //clear flag on start
        SmartDashboard.putBoolean("RTDrive OK", true);
        firstCycle = true;
        while (!exitFlag) {
            long start = System.nanoTime();
            
            advanceTracking();

            if(driverStick.getRawButtonPressed(calibrateButton)) {
                calibrateTracking();
            }

            //alignEnabled = driverStick.getRawButton(4);
            if(driverStick.getRawButton(autoButton) && alignDCOK/*replace with housekeeping*/ ) { //drive takeover, make sure alignDC is ok
                //following is placeholder
                float minSpeed = 0.06f;
                delta = 0.0;
                double maxTurning = 0.3;
                double deltaT;
                switch(mode) {
                    case rotate:
                        delta = targetAngle - (realyaw - targetAngleOffset);

                        deltaT = (System.nanoTime() - angleTS) / 1000000;
                        angleTS = System.nanoTime();

                        eIntegral += delta * deltaT;
                        SmartDashboard.putNumber("eIntegral", eIntegral);

                        aimInput = (delta * angleP) + (eIntegral * angleI);

                        aimInput = (aimInput > maxTurning)? maxTurning: aimInput;
                        aimInput = (aimInput < -maxTurning)? -maxTurning: aimInput;
                        //
                        aimInputy = 0;

                        autoConditionSatisfied = (Math.abs(delta) < 1.0) && (Math.abs(navX.getRate()) < 0.02); //auto is satisfied if almost still
                        break;
                    case distance:
                        delta = targetDistance - ((((leftPosition - leftOffset)+(rightPosition - rightOffset))/2) / ticksPerCentimetre);
                        aimInput = 0;
                        aimInputy = delta * distanceP;
                        autoConditionSatisfied = (Math.abs(delta) < 4.0);
                        break;
                    case curve:
                        double angleRateP = 0.5;
                        double angleRateI = 0.01;

                        double progress = ((((leftPosition - leftOffset)+(rightPosition - rightOffset))/2) / ticksPerCentimetre) / targetDistance; //0 to 1 of the distance we have traveled
                        
                        double distanceDelta = targetDistance - ((((leftPosition - leftOffset)+(rightPosition - rightOffset))/2) / ticksPerCentimetre);
                        aimInputy = distanceDelta * distanceP;

                        double rateDelta = targetGyroRate - (navX.getRate());

                        deltaT = (System.nanoTime() - angleTS) / 100000000;
                        angleTS = System.nanoTime();

                        eIntegral += rateDelta * deltaT;

                        aimInput = (rateDelta * angleRateP) + (eIntegral * angleRateI);

                        aimInput = (aimInput > maxTurning)? maxTurning: aimInput;
                        aimInput = (aimInput < -maxTurning)? -maxTurning: aimInput;
                        break;
                    case stop:
                        aimInput = 0;
                        aimInputy = 0;
                }
                

                aimInput = (aimInput > 1)? 1 : aimInput;
                aimInputy = (aimInputy > 0.3)? 0.3 : aimInputy;

                aimInput = (aimInput < minSpeed && aimInput > 0)? minSpeed : aimInput;
                aimInput = (aimInput > -minSpeed && aimInput < 0)? -minSpeed : aimInput;

                leftDrive = aimInput + aimInputy;
                rightDrive = -aimInput + aimInputy;

                DL1Motor.set(ControlMode.PercentOutput, leftDrive);
                DL2Motor.set(ControlMode.PercentOutput, leftDrive);
                DR1Motor.set(ControlMode.PercentOutput, -rightDrive); //inverting
                DR2Motor.set(ControlMode.PercentOutput, -rightDrive);
                
            } else { //run the regular drive TODO: drive calculations
                double deadZone = 0.2;
                double fullSpeed = 1.0;


                //     TriggerDrive(driveStick.getRawAxis(0), driveStick.getRawAxis(2), driveStick.getRawAxis(3));
                double Lt;
                double Rt;
                if(controllerType == ControllerType.PS5) {
                    Lt = (driverStick.getRawAxis(2) + 1.0) / 2;
                    Rt = (driverStick.getRawAxis(5) + 1.0) / 2;
                } else {
                    Lt = driverStick.getRawAxis(3);
                    Rt = driverStick.getRawAxis(2);
                }

                if(Lt < 0 || Rt < 0) {
                    forcefulDisconnect("invalid trigger inputs " + Lt + " " + Rt);
                    exitFlag = true;
                    return;
                }

                double y = Rt - Lt;

                //double y = driverStick.getY() * -1;
                double x = driverStick.getX();
                
                //deadzone calclations
                x = (x < deadZone && x > -deadZone)? 0 : x;
                if(x != 0.0) x = (x > 0.0)? x - deadZone : x + deadZone; //eliminate jump behaviour
                x = x / (1 - deadZone);
                simOut("xval", x);

                double aparam = 0.6;

                x = (aparam * (x * x * x)) + ((1-aparam) * x);
                
                y = (y < deadZone && y > -deadZone)? 0 : y;
                if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                y = y / (1 - deadZone);
                simOut("yval", y);

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
                        forcefulDisconnect("manual drive state not zeroed on startup");
                        exitFlag = true;
                        return;
                    }
                    firstCycle = false;
                }

                DL1Motor.set(ControlMode.PercentOutput, leftDrive);
                DL2Motor.set(ControlMode.PercentOutput, leftDrive);
                DR1Motor.set(ControlMode.PercentOutput, -rightDrive);
                DR2Motor.set(ControlMode.PercentOutput, -rightDrive);
                
                delta = realyaw - targetAngleOffset;
            }

            
            
            //shooterEnabled = driverStick.getRawButton(1);
            //simOut("shooterEnabled", shooterEnabled);
            /*if (shooterEnabled) {
                //set the shooter motor
                if (alignDCOK) {shooterMotor.set(ControlMode.PercentOutput, shooterInput); simOut("shooterTarget", shooterInput);} else {
                    //failsafe needs to be updated
                    shooterMotor.set(ControlMode.PercentOutput, 0.5);
                    simOut("shooterTarget", shooterInput);
                }
            } else {
                shooterInput = 0;
                simOut("shooterTarget", shooterInput);
                shooterMotor.set(ControlMode.PercentOutput, 0.0);
            }*/
            
            long elapsedTime = System.nanoTime() - start;
            simOut("RTDrive last time", (double)elapsedTime);
            if(elapsedTime > 1000000) {
                //System.out.println("[RTDrive] Motion processing took longer than 1ms! Took " + elapsedTime + "uS");
            }


            //TODO: improve RTDrive thread timing
            //try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
        SmartDashboard.putBoolean("RTDrive OK", false);
        System.out.println("[RTDrive] left independent service");
    }
}

