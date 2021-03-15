package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;

import edu.wpi.first.wpilibj.Joystick;

public class AlignDriveCamera implements Runnable {
    //limelight inputs
    NetworkTableEntry tp; //target present
    NetworkTableEntry tx; //target x
    NetworkTableEntry ty; //target y
    double targetHeight = 7.0; //height off ground

    RealTimeDrive RTDrive;

    long checkIn;

    Joystick driveStick;

    public AlignDriveCamera(RealTimeDrive RTDrive, Joystick driveStick) {
        this.RTDrive = RTDrive;
        this.driveStick = driveStick;
    }
    
    public void initCamera() { //setup nettables for the camera
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

        tp = table.getEntry("tp");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        if(!RobotBase.isReal()) { //setup values
            tp.setBoolean(false);
            tx.setDouble(0.0);
            ty.setDouble(0.0);
        }
    }
    
    double calcDistance(double camY) {
        double fov = 49.7; //vertical for limelight 2+
        double angle = ((camY + 1.0) / 2) * fov;
        angle = angle + 90;
        //circle math
        double radius = 1.0;
		double radians = Math.toRadians(angle);
		double tpy = radius * Math.sin(radians);
		double tpx = radius * Math.cos(radians);
		//calculate slope ratio
		double slope = (tpx / tpy); 
        //x = (y - height) / sl
        //NOTE: Target height is offset above robot
		double x_intersect = (0 - targetHeight) / slope;
		if (x_intersect < 6.0) {
			x_intersect = 6.0; //? cant remember why, might remove
		}
		return x_intersect;
    }

    public boolean exitFlag = false;
    public void run() {
        exitFlag = false;
        System.out.println("AlignDC: Start");
        //intakeLiftMotor.set(ControlMode.PercentOutput ,0.1);
        while(!exitFlag) {
            long start = System.currentTimeMillis();//marker for exec timing
            
            if (driveStick.getRawButton(1) && tp.getBoolean(false)) {
                //TODO: Shooter math
                //RTDrive.shooterInput = this.calcDistance(ty.getDouble(0.0)); //velocity
                if (!(RTDrive.getShooterVelocity() > RTDrive.shooterInput)) { //semi placeholder, needs target
                    //TODO: shooter verification
                } else {
                    //shooter velocity bad
                }
            }
            
            if (RTDrive.alignEnabled) {
                //TODO: PID loop for alignment
                //this needs to be a PID loop based on the 
            }
            checkIn = System.currentTimeMillis();
            //throws InterruptedException {Thread.sleep(10);}
            if(System.currentTimeMillis() < (start + 1)) try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
    }
}
