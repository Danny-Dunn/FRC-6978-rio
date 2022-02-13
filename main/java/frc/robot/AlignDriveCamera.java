package frc.robot;


import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import org.opencv.ml.RTrees;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RealTimeDrive.DriveMode;
import edu.wpi.first.wpilibj.Joystick;



public class AlignDriveCamera implements Runnable {
    //limelight inputs
    NetworkTableEntry tp; //target present
    NetworkTableEntry tx; //target x
    NetworkTableEntry ty; //target y
    double targetHeight = 7.0; //height off ground

    long pidXTimeStamp;
    long pidYTimeStamp;
    boolean pidPreviousState;

    RealTimeDrive RTDrive;

    long checkIn;

    Joystick driveStick;

    public TalonSRX shooterMotor;
    TalonSRX loaderMotor;
    TalonSRX intakeLiftMotor;

    double startTimeStamp;

    int autoState;
    int pointNum;
    /*Vector2d points[] = {
        new Vector2d(0, 200),
        new Vector2d(100, 200),
        new Vector2d(0, 200),
        new Vector2d(0, 0)
    };*/
    Vector2d points[] = {
        new Vector2d(200, 0),
        new Vector2d(300, -100),
        new Vector2d(300, 100),
        new Vector2d(200, 0),
        new Vector2d(0, 0)
    };
    /*Vector2d points[] = {
        new Vector2d(0, 200),
        new Vector2d(200, 200),
        new Vector2d(200, 0),
        new Vector2d(0, 0)
    };*/
    double dx;
    double dy;

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

        autoState = 0;
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
            
            if (driveStick.getRawButton(8)) {
                //TODO: Full shooter distance calculation
                //RTDrive.shooterInput = this.calcDistance(ty.getDouble(0.0)); //velocity
                //shooterMotor.set(ControlMode.Velocity, 2300); //works at 2300 zone 3 actual 2200
                //shooterMotor.set(ControlMode.PercentOutput, 0.35); //testing
                //shooterMotor.set(conr)
                
            } /*else if (driveStick.getRawButton(2)) {
                shooterMotor.set(ControlMode.Velocity, 1000);
            } else if (driveStick.getRawButton(5)) {
                shooterMotor.set(ControlMode.Velocity, 3150);
            }*/ else {
                //shooterMotor.set(ControlMode.PercentOutput, 0);
                
            }
            
            if (driveStick.getRawButton(3)) {
                //RTDrive.shooterInput = this.calcDistance(ty.getDouble(0.0)); //velocity
                //loaderMotor.set(ControlMode.PercentOutput, 0.4);
                
            } else {
                //loaderMotor.set(ControlMode.PercentOutput, 0);
                
            }

            /*if (driveStick.getRawButton(7)) {
                double error = tx.getDouble(0) - 6.24;
                SmartDashboard.putNumber("Xerror", error);
                double gain = 0.045;
                double gainI = 0.02;
                double motorOut;
                double i;
                double p;

                if(!pidPreviousState) { //reset on new button
                    pidXTimeStamp = System.currentTimeMillis();
                    pidPreviousState = true;
                }

                if(Math.abs(error) < 0.2) { //reset on align
                    pidXTimeStamp = System.currentTimeMillis();
                }

                i = (System.currentTimeMillis() - startTimeStamp) * gainI;

                /*if(error > 0.4) {
                    p = error * gain;
                } else { 
                    p = error * gain2;
                }
                p = error * gain;

                motorOut = i + p;
                
                RTDrive.aimInputy = -motorOut;
                SmartDashboard.putNumber("camXPresult", p);
                SmartDashboard.putNumber("camXIresult", i);
            }

            if (driveStick.getRawButton(7)) {
                double error = ty.getDouble(0);
                SmartDashboard.putNumber("Yerror", error);
                double gain = 0.045;
                double gainI = 0.02;
                double motorOut;
                double i;
                double p;

                if(!pidPreviousState) { //reset on new button
                    pidYTimeStamp = System.currentTimeMillis();
                    pidPreviousState = true;
                }

                if(Math.abs(error) < 0.2) { //reset on align
                    pidYTimeStamp = System.currentTimeMillis();
                }

                i = (System.currentTimeMillis() - startTimeStamp) * gainI;

                /*if(error > 0.4) {
                    p = error * gain;
                } else { 
                    p = error * gain2;
                }
                p = error * gain;

                motorOut = i + p;
                
                //RTDrive.aimInputy = -motorOut;
                SmartDashboard.putNumber("camYPresult", p);
                SmartDashboard.putNumber("camYIresult", i);

            }*/

            if(driveStick.getRawButtonPressed(7)) {
                autoState = 0;
                pointNum = 0;
                RTDrive.setDriveMode(DriveMode.stop);
            } 
            
            if(driveStick.getRawButton(7)) {//test auto code
                SmartDashboard.putNumber("autoState", autoState);
                SmartDashboard.putNumber("targetPOint", pointNum);
                switch(autoState) {
                    case 0:
                        if(pointNum > points.length - 1) {
                            autoState = -1; //shut down autodrive
                        } else {
                            dy = points[pointNum].y - RTDrive.currentPosition.y;
                            dx = points[pointNum].x - RTDrive.currentPosition.x;
                            if(Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0)) < 6.0) { //check if we are already close enough
                                pointNum++;
                            } else {
                                autoState++;
                            }
                        }
                    case 1: //rotate to angle
                        dy = points[pointNum].y - RTDrive.currentPosition.y;
                        dx = points[pointNum].x - RTDrive.currentPosition.x;
                        RTDrive.targetAngle = Math.toDegrees(Math.atan2(dy, dx));
                        
                        RTDrive.setDriveMode(DriveMode.rotate);
                        autoState++;
                        break;
                    case 2:
                        if(RTDrive.autoConditionSatisfied) {
                            autoState++;
                        }
                        break;
                    case 3:
                        dy = points[pointNum].y - RTDrive.currentPosition.y;
                        dx = points[pointNum].x - RTDrive.currentPosition.x;
                        RTDrive.targetDistance = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));
                        RTDrive.setDriveMode(DriveMode.distance);
                        autoState++;
                        break;
                    case 4:
                        if(RTDrive.autoConditionSatisfied) {
                            dy = points[pointNum].y - RTDrive.currentPosition.y;
                            dx = points[pointNum].x - RTDrive.currentPosition.x;
                            if(Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0)) < 15.0) {
                                autoState = 0;
                                pointNum++;
                            } else {
                                autoState = 1;
                            }
                        }
                    break;
                }
                SmartDashboard.putNumber("DeltaX", dx);
                SmartDashboard.putNumber("DeltaY", dy);
            }
            SmartDashboard.putNumber("autostate", autoState);

            //SmartDashboard.putNumber("shooterVelocity", shooterMotor.getSelectedSensorVelocity());
            //SmartDashboard.putNumber("shooterCurrent", shooterMotor.getStatorCurrent());

            checkIn = System.currentTimeMillis();
            //throws InterruptedException {Thread.sleep(10);}
            //if(System.currentTimeMillis() < (start + 1)) try {Thread.sleep(1);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
        intakeLiftMotor.set(ControlMode.PercentOutput, 0);
    }
}
