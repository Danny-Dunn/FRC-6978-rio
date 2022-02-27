package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RealTimeDrive.DriveMode;
import frc.robot.AutoCommand.CommandType;
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
    /*Vector2d points[] = {
        new Vector2d(200, 0),
        new Vector2d(300, -100),
        new Vector2d(300, 100),
        new Vector2d(200, 0),
        new Vector2d(0, 0)
    };*/
    Vector2d points[] = {
        new Vector2d(92, 0),
        new Vector2d(138, -45),
        new Vector2d(330, -52),
        new Vector2d(335, 37),
        new Vector2d(275, 99),
        new Vector2d(137, 115),
        new Vector2d(101, 0),
        new Vector2d(0, 0),
    };

    AutoCommand commands[] = {
        new AutoCommand(CommandType.SetIntake, 100, 0),
        new AutoCommand(82, 0),
        new AutoCommand(CommandType.SetIntake, 100, 0.6),
        new AutoCommand(96, 0),
        new AutoCommand(CommandType.SetIntake, 0, 0),
        new AutoCommand(CommandType.RotateToAngle, 180, 0),
        new AutoCommand(CommandType.SetShooter, 14000, 2), //fire 2 balls
        new AutoCommand(430, 146), //go near driver station
    };

    /*AutoCommand commands[] = {
        new AutoCommand(100, 0),
        new AutoCommand(0, 0)
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

        autoState = 0;
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

            if(driveStick.getRawButtonPressed(7)) {
                autoState = 0;
                pointNum = 0;
                RTDrive.setDriveMode(DriveMode.stop);
                System.out.println("[AlignDC] reset auto system");
            } 
            
            SmartDashboard.putNumber("autoState", autoState);
            SmartDashboard.putNumber("targetPOint", pointNum);
            SmartDashboard.putString("driveMode", RTDrive.mode.toString());

            if(driveStick.getRawButton(7)) {//test auto code
                
                switch(autoState) {
                    case 0:
                        if(pointNum > commands.length) {
                            autoState = -1;
                            break;
                        }
                        switch(commands[pointNum].type) {
                            case DriveToPoint:
                                autoState = 1;
                                break;
                            case SetIntake:
                                //set intake
                                pointNum++;
                                break;
                            case SetShooter:
                                //set shooter
                                pointNum++;
                                break;
                            case RotateToAngle:
                                autoState = 2;
                                break;
                            default:
                                pointNum++;
                                break;
                        }
                        break;
                    case 1: // start driving to point
                        
                        dy = commands[pointNum].point.y - RTDrive.currentPosition.y;
                        dx = commands[pointNum].point.x - RTDrive.currentPosition.x;
                        if(Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0)) < 6.0) { //check if we are already close enough
                            pointNum++;
                            autoState = 0;
                        } else {
                            autoState++;
                        }
                        
                        break;
                    case 2: //rotate to angle
                        switch(commands[pointNum].type) {
                            case DriveToPoint:
                                dy = commands[pointNum].point.y - RTDrive.currentPosition.y;
                                dx = commands[pointNum].point.x - RTDrive.currentPosition.x;
                                RTDrive.targetAngle = Math.toDegrees(Math.atan2(dy, dx));
                                RTDrive.setDriveMode(DriveMode.rotate);
                                autoState++;
                                break;
                            case RotateToAngle:
                                System.out.println("[AlignDC] rot to angle");
                                RTDrive.targetAngle = commands[pointNum].aparam; 
                                RTDrive.setDriveMode(DriveMode.rotate);
                                autoState++;
                                break;
                            default:
                                autoState = 0;
                                break;

                        }
                        
                        break;
                    case 3:
                        if(RTDrive.autoConditionSatisfied) {
                            switch(commands[pointNum].type) {
                                case DriveToPoint:
                                    autoState++;
                                    break;
                                default:
                                    autoState = 0;
                                    pointNum++;
                                    break;

                            }
                            
                        }
                        break;
                    case 4:
                        dy = commands[pointNum].point.y - RTDrive.currentPosition.y;
                        dx = commands[pointNum].point.x - RTDrive.currentPosition.x;
                        RTDrive.targetDistance = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));
                        RTDrive.setDriveMode(DriveMode.distance);
                        autoState++;
                        break;
                    case 5:
                        if(RTDrive.autoConditionSatisfied) {
                            dy = commands[pointNum].point.y - RTDrive.currentPosition.y;
                            dx = commands[pointNum].point.x - RTDrive.currentPosition.x;
                            if(Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0)) < 15.0) {
                                pointNum++;
                                autoState = 0;
                            } else {
                                autoState = 2;
                            }
                            RTDrive.setDriveMode(DriveMode.stop);
                        }
                        break;
                    default:
                        break;
                    
                }
                SmartDashboard.putNumber("DeltaX", dx);
                SmartDashboard.putNumber("DeltaY", dy);
            }

            checkIn = System.currentTimeMillis();
            //throws InterruptedException {Thread.sleep(10);}
            try {Thread.sleep(3);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
    }
}
