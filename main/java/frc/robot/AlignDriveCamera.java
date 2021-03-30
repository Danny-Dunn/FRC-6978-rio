package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    public TalonSRX shooterMotor;
    TalonSRX loaderMotor;
    TalonSRX intakeLiftMotor;

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

        shooterMotor = new TalonSRX(24);
        loaderMotor = new TalonSRX(25);

        intakeLiftMotor = new TalonSRX(16);

        shooterMotor.setInverted(true);
        loaderMotor.setInverted(true);
        shooterMotor.setSensorPhase(false);

        shooterMotor.config_kP(0, 14);
        //SmartDashboard.putNumber("shooterP", 7);
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
        //intakeLiftMotor.set(ControlMode.PercentOutput, 0.1);
        exitFlag = false;
        System.out.println("AlignDC: Start");
        //intakeLiftMotor.set(ControlMode.PercentOutput ,0.1);
        while(!exitFlag) {
            long start = System.currentTimeMillis();//marker for exec timing
            
            if (driveStick.getRawButton(6)) {
                //TODO: Shooter math
                //RTDrive.shooterInput = this.calcDistance(ty.getDouble(0.0)); //velocity
                //shooterMotor.set(ControlMode.Velocity, 2300); //works at 2300 zone 3 actual 2200
                shooterMotor.set(ControlMode.Velocity, 2700); //testing
                //shooterMotor.set(conr)
                
            } else if (driveStick.getRawButton(2)) {
                shooterMotor.set(ControlMode.Velocity, 1000);
            } else if (driveStick.getRawButton(5)) {
                shooterMotor.set(ControlMode.Velocity, 3150);
            } else {
                shooterMotor.set(ControlMode.PercentOutput, 0);
                
            }
            
            if (driveStick.getRawButton(3)) {
                //TODO: Shooter math
                //RTDrive.shooterInput = this.calcDistance(ty.getDouble(0.0)); //velocity
                loaderMotor.set(ControlMode.PercentOutput, 0.4);
                
            } else {
                loaderMotor.set(ControlMode.PercentOutput, 0);
                
            }

            if (driveStick.getRawButton(8)) {
                double error = tx.getDouble(0) - 6.24;
                SmartDashboard.putNumber("error", error);
                double gain = 0.045;
                double gain2 = 0.025;
                double motorOut;

                if(error > 0.4) {
                    motorOut = error * gain;
                } else { 
                    motorOut = error * gain2;
                }
                if(Math.abs(error) < 0.3) {
                    error = 0;
                }
                RTDrive.aimInput = motorOut;
            }

            SmartDashboard.putNumber("shooterVelocity", shooterMotor.getSelectedSensorVelocity());
            SmartDashboard.putNumber("shooterCurrent", shooterMotor.getStatorCurrent());

            checkIn = System.currentTimeMillis();
            //throws InterruptedException {Thread.sleep(10);}
            if(System.currentTimeMillis() < (start + 1)) try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
        intakeLiftMotor.set(ControlMode.PercentOutput, 0);
    }
}
