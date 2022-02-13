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
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.SPI;

//simOuts are for simulation purposes only, these can be removed in final robot code to improve efficiency

public class RealTimeDrive implements Runnable {
    //motors
    TalonFX DL1Motor;
    TalonFX DL2Motor;
    TalonFX DR1Motor;
    TalonFX DR2Motor;
    public TalonSRX shooterMotor;
    TalonSRX loaderMotor;
    
    Joystick driverStick;
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
        stop
    };

    DriveMode mode;
    double targetAngle;
    double targetAngleOffset;
    double targetDistance;
    double leftOffset;
    double rightOffset;
    boolean autoConditionSatisfied;

    //SmartDashboard sdb;

    double angleP;
    double angleP2;
    double distanceP;

    void setDriveMode(DriveMode dm) {
        mode = dm;
        autoConditionSatisfied = false;
        switch(dm) {
            case rotate:
                targetAngleOffset = realyaw;
                //targetAngle = targetAngle - targetAngleOffset;
                double absDelta = targetAngle - absyaw;
                if(absDelta > 180) {
                    absDelta = absDelta - 360;
                } else if(absDelta < -180) {
                    absDelta = absDelta + 360;
                }
                targetAngle = absDelta;
                System.out.println("Turning " + targetAngle);
                SmartDashboard.putNumber("targetOffset", targetAngleOffset);
                SmartDashboard.putNumber("absdelta", absDelta);
                break;
            case distance:
                leftOffset = leftPosition;
                rightOffset = rightPosition;
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
        absyaw = realyaw % 360;
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
        SmartDashboard.putNumber("angleP", 0.0018);
        SmartDashboard.putNumber("angleP2", 0.0144);
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
        
        calibrateTracking();
        System.out.println("[RTDrive] Calibrated tracking with angle offset " + angleOffset);

        System.out.println("[RTDrive] Initialised module");

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
            angleP = SmartDashboard.getNumber("angleP", 0.002);
            angleP2 = SmartDashboard.getNumber("angleP2", 0.002);

            distanceP = SmartDashboard.getNumber("distanceP", 0.002);
        }
    }

    public boolean exitFlag;
    public void run() { //might remove
        
        exitFlag = false; //clear flag on start
        SmartDashboard.putBoolean("RTDrive OK", true);
        while (!exitFlag) {
            long start = System.nanoTime();
            
            advanceTracking();
            if(driverStick.getRawButtonPressed(8)) {
                calibrateTracking();
            }

            //alignEnabled = driverStick.getRawButton(4);
            if(driverStick.getRawButton(7) && alignDCOK/*replace with housekeeping*/ ) { //drive takeover, make sure alignDC is ok
                //following is placeholder
                float minSpeed = 0.06f;
                delta = 0.0;
                double maxTurning = 0.6;
                switch(mode) {
                    case rotate:
                        delta = targetAngle - (realyaw - targetAngleOffset);

                        if(delta < 130.0) {
                            aimInput = delta * angleP2;
                        } else {
                            aimInput = delta * angleP;
                        }
                        aimInput = (aimInput > maxTurning)? maxTurning: aimInput;
                        aimInput = (aimInput < -maxTurning)? -maxTurning: aimInput;
                        //
                        aimInputy = 0;

                        autoConditionSatisfied = (Math.abs(delta) < 1.0) && (navX.getRate() < 0.005); //auto is satisfied if almost still
                        break;
                    case distance:
                        delta = targetDistance - ((((leftPosition - leftOffset)+(rightPosition - rightOffset))/2) / ticksPerCentimetre);
                        aimInput = 0;
                        aimInputy = delta * distanceP;
                        autoConditionSatisfied = (Math.abs(delta) < 4.0);
                }
                

                aimInput = (aimInput > 1)? 1 : aimInput;
                aimInputy = (aimInputy > 0.5)? 0.5 : aimInputy;

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
                double fullSpeed = 0.95;


                //     TriggerDrive(driveStick.getRawAxis(0), driveStick.getRawAxis(2), driveStick.getRawAxis(3));
                double Lt = driverStick.getRawAxis(3);
                double Rt = driverStick.getRawAxis(2);
                double y = Lt  - Rt;

                //double y = driverStick.getY() * -1;
                double x = driverStick.getX();
                double aparam = 0.1;
                double bparam = 0.8;
                x = (aparam * (x * x * x)) + (bparam * x);
                //deadzone calclations
                x = (x < deadZone && x > -deadZone)? 0 : x;
                if(x != 0.0) x = (x > 0.0)? x - deadZone : x + deadZone; //eliminate jump behaviour
                simOut("xval", x);
                x = x * 0.7;
                
                y = (y < deadZone && y > -deadZone)? 0 : y;
                if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                simOut("yval", y);

                leftDrive = y + x;
                rightDrive = y - x;
                
                leftDrive = leftDrive / (1.0 - deadZone);
                rightDrive = rightDrive / (1.0 - deadZone);
                //speed scaling
                leftDrive = leftDrive * fullSpeed; 
                rightDrive = rightDrive * fullSpeed;
                //speed hard cap
                leftDrive = (leftDrive > fullSpeed)? fullSpeed : leftDrive;
                rightDrive = (rightDrive > fullSpeed)? fullSpeed : rightDrive;
                leftDrive = (leftDrive < -fullSpeed)? -fullSpeed : leftDrive;
                rightDrive = (rightDrive < -fullSpeed)? -fullSpeed : rightDrive;

                  
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
            try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
        SmartDashboard.putBoolean("RTDrive OK", false);
    }
}

