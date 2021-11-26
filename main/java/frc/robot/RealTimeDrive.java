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
    double oldYaw;
    double ticksPerCentimetre = 385.47; //hopefully correct
    AHRS navX;

    //SmartDashboard sdb;

    Vector2d calcGraphTransition(Vector2d lastPosition, double distance, double yaw) {
		double radius = (distance) / ticksPerCentimetre;
		double radians = Math.toRadians(yaw);
		//calc the new point
        lastPosition.y = lastPosition.y + (radius * Math.sin(radians));
        lastPosition.x = lastPosition.x + (radius * Math.cos(radians));
        return lastPosition;
	}

    void advanceTracking() {
        double leftPosition = DR1Motor.getSelectedSensorPosition();
        double rightPosition = DR1Motor.getSelectedSensorPosition();
        double yaw = navX.getYaw();
        currentPosition = calcGraphTransition(currentPosition, ((leftPosition - oldLeftDrivePosition) + (rightPosition - oldRightDrivePosition)) / 2, (yaw + oldYaw) / 2);
        oldYaw = yaw;
        oldLeftDrivePosition = leftPosition;
        oldRightDrivePosition = rightPosition;
        //TODO: Maybe remove these as they may cost performance
        SmartDashboard.putNumber("trackX", currentPosition.x);
        SmartDashboard.putNumber("trackY", currentPosition.y);
    }

    void calibrateTracking() {
        navX.calibrate();
    }

    public RealTimeDrive(Joystick driveStick) {
        //meta stuff
        this.driverStick = driveStick;
        this.aimInput = 0;
    }
    //output functions for simulation
    public void simOut(String tag, Double value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setDouble(value);
        }
    }
    public void simOut(String tag, Boolean value) {
        if (!RobotBase.isReal()) {
            simTable.getEntry(tag).setBoolean(value);
        }
    }

    public double getShooterVelocity() {
        return shooterMotor.getSelectedSensorVelocity();
    }

    public boolean setup() {
        //setup motors
        DL1Motor = new TalonFX(1);
        DL2Motor = new TalonFX(2);
        DR1Motor = new TalonFX(3);
        DR2Motor = new TalonFX(4);

        if(!RobotBase.isReal()) simTable = NetworkTableInstance.getDefault().getTable("simTable"); //simulation dummy outputs
        System.out.println("[RtDrive] Start OK");

        currentPosition = new Vector2d(0, 0);
        navX = new AHRS(SPI.Port.kMXP);

        return true; //everything went fine
    }
    
    public boolean exitFlag;
    public void run() { //might remove
        
        exitFlag = false; //clear flag on start
        SmartDashboard.putBoolean("RTDrive OK", true);
        while (!exitFlag) {
            long start = System.nanoTime();
            
            advanceTracking();

            //alignEnabled = driverStick.getRawButton(4);
            if(driverStick.getRawButton(7) && alignDCOK/*replace with housekeeping*/ ) { //drive takeover, make sure alignDC is ok
                //following is placeholder
                float minSpeed = 0.08f;
                aimInput = (aimInput > 0.3)? 0.3 : aimInput;
                aimInputy = (aimInputy > 0.25)? 0.25 : aimInputy;

                aimInput = (aimInput < minSpeed && aimInput > 0)? minSpeed : aimInput;
                aimInput = (aimInput > -minSpeed && aimInput < 0)? -minSpeed : aimInput;

                double leftDrive = aimInput + aimInputy;
                double rightDrive = -aimInput + aimInputy;

                DL1Motor.set(ControlMode.PercentOutput, leftDrive);
                DL2Motor.set(ControlMode.PercentOutput, leftDrive);
                DR1Motor.set(ControlMode.PercentOutput, -rightDrive); //inverting
                DR2Motor.set(ControlMode.PercentOutput, -rightDrive);
                simOut("leftDrive", leftDrive);
                simOut("rightDrive", rightDrive);
            } else { //run the regular drive TODO: drive calculations
                double deadZone = 0.2;
                double fullSpeed = 0.5;

                double y = driverStick.getY() * -1;
                double x = driverStick.getX();
                //deadzone calclations
                x = (x < deadZone && x > -deadZone)? 0 : x;
                if(x != 0.0) x = (x > 0.0)? x - deadZone : x + deadZone; //eliminate jump behaviour
                simOut("xval", x);
                x = x * 0.7;
                
                y = (y < deadZone && y > -deadZone)? 0 : y;
                if(y != 0.0) y = (y > 0.0)? y - deadZone : y + deadZone; //eliminate jump behaviour
                simOut("yval", y);

                double leftDrive = y + x;
                double rightDrive = y - x;
                
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
                //TODO: Determine if detailed SmartDasboard output is ok on performance
                simOut("leftDrive", leftDrive);
                simOut("rightDrive", rightDrive);
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
                System.out.println("[RTDrive] Motion processing took longer than 1ms! Took " + elapsedTime + "uS");
            }

            //TODO: improve RTDrive thread timing
            try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
        SmartDashboard.putBoolean("RTDrive OK", false);
    }
}

