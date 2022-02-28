package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RealTimeDrive.AutoMode;
import frc.robot.AutoCommand.CommandType;



public class AutonomousController implements Runnable, ServiceableModule {
    Thread mThread;
    
    //limelight inputs
    NetworkTableEntry tp; //target present
    NetworkTableEntry tx; //target x
    NetworkTableEntry ty; //target y
    double targetHeight = 7.0; //height off ground

    long pidXTimeStamp;
    long pidYTimeStamp;
    boolean pidPreviousState;

    RealTimeDrive mRealTimeDrive;
    Intake mIntake;

    long checkIn;

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
        new AutoCommand(CommandType.SetIntake, 500, 0),
        new AutoCommand(82, 0),
        new AutoCommand(CommandType.SetIntake, 500, 0.75),
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

    public AutonomousController(RealTimeDrive RTDrive, Intake intake) {
        mRealTimeDrive = RTDrive;
        mIntake = intake;
    }
    
    public boolean init() { //setup nettables for the camera
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
        return true;
    }
    
    public boolean start() {
        exitFlag = false;

        autoState = 0;
        pointNum = 0;

        mThread = new Thread(this, "AutonomousController");
        mThread.start();

        return true;
    }

    public boolean stop() {
        if(mThread == null) return true;
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

        return true;
    }

    public void standby(boolean takeInputs) {
        return;
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
        System.out.println("[AutonomousController] Entered independent service");
        while(!exitFlag) {
            
            SmartDashboard.putNumber("autoState", autoState);
            SmartDashboard.putNumber("targetPoint", pointNum);
            //SmartDashboard.putString("driveMode", mRealTimeDrive.mAutoMode.toString());

            
                
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
                            mIntake.setLift(commands[pointNum].aparam);
                            mIntake.setRollers(commands[pointNum].bparam);
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
                    
                    dy = commands[pointNum].point.y - mRealTimeDrive.currentPosition.y;
                    dx = commands[pointNum].point.x - mRealTimeDrive.currentPosition.x;
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
                            dy = commands[pointNum].point.y - mRealTimeDrive.currentPosition.y;
                            dx = commands[pointNum].point.x - mRealTimeDrive.currentPosition.x;
                            mRealTimeDrive.targetAngle = Math.toDegrees(Math.atan2(dy, dx));
                            mRealTimeDrive.setDriveMode(AutoMode.rotate);
                            autoState++;
                            break;
                        case RotateToAngle:
                            System.out.println("[AlignDC] rot to angle");
                            mRealTimeDrive.targetAngle = commands[pointNum].aparam; 
                            mRealTimeDrive.setDriveMode(AutoMode.rotate);
                            autoState++;
                            break;
                        default:
                            autoState = 0;
                            break;

                    }
                    
                    break;
                case 3:
                    if(mRealTimeDrive.autoConditionSatisfied) {
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
                    dy = commands[pointNum].point.y - mRealTimeDrive.currentPosition.y;
                    dx = commands[pointNum].point.x - mRealTimeDrive.currentPosition.x;
                    mRealTimeDrive.targetDistance = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));
                    mRealTimeDrive.setDriveMode(AutoMode.distance);
                    autoState++;
                    break;
                case 5:
                    if(mRealTimeDrive.autoConditionSatisfied) {
                        dy = commands[pointNum].point.y - mRealTimeDrive.currentPosition.y;
                        dx = commands[pointNum].point.x - mRealTimeDrive.currentPosition.x;
                        if(Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0)) < 15.0) {
                            pointNum++;
                            autoState = 0;
                        } else {
                            autoState = 2;
                        }
                        mRealTimeDrive.setDriveMode(AutoMode.stop);
                    }
                    break;
                case 6: //await intake
                    if(mIntake.getAutoConditionSatisfied()) {
                        pointNum++;
                        autoState = 0;
                    } 
                    break;
                default:
                    break;
                
            }
            SmartDashboard.putNumber("DeltaX", dx);
            SmartDashboard.putNumber("DeltaY", dy);
            

            checkIn = System.currentTimeMillis();
            //throws InterruptedException {Thread.sleep(10);}
            try {Thread.sleep(3);} catch (InterruptedException ie) {} //prevents the thread from running too fast
        }
        System.out.println("[AutonomousController] Left independent service");
    }
}
