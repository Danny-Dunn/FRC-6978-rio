package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RealTimeDrive.AutoMode;
import frc.robot.Shooter.ShooterControlMode;
import frc.robot.AutoCommand.CommandType;
import frc.robot.Intake.RollerMode;



public class AutonomousController implements Runnable, ServiceableModule {
    Thread mThread;
    
    //limelight inputs
    NetworkTableEntry tp; //target present
    NetworkTableEntry tx; //target x
    NetworkTableEntry ty; //target y

    long pidXTimeStamp;
    long pidYTimeStamp;
    boolean pidPreviousState;

    RealTimeDrive mRealTimeDrive;
    Intake mIntake;
    Shooter mShooter;

    long checkIn;

    private long delayTS;
    private long currentDelay;

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

    private int autoSequence = 2;

    AutoCommand commands[][] = {
        {
            new AutoCommand(CommandType.SetIntake, 950, 0.5),
            new AutoCommand(CommandType.Delay, 1000, 0),
            new AutoCommand(152, 0),
            new AutoCommand(CommandType.SetIntake, 950, 0.5),
            new AutoCommand(CommandType.SetIntake, -82, 0),
            new AutoCommand(CommandType.Delay, 500, 0),
            new AutoCommand(CommandType.RotateToAngle, 180, 0),
            new AutoCommand(CommandType.SetShooter, 1, 0), 
            new AutoCommand(0, 0),
            new AutoCommand(CommandType.SetShooter, 1, 1), 
            new AutoCommand(CommandType.Delay, 3000, 0), //wait while it fires
            new AutoCommand(CommandType.SetShooter, 0, 0),
            new AutoCommand(330,-100), //go near driver station
        },

        {
            new AutoCommand(CommandType.SetShooter, 1, 1), 
            new AutoCommand(CommandType.Delay, 1500, 0), //wait while it fires
            new AutoCommand(CommandType.SetShooter, 0, 0),
            new AutoCommand(CommandType.DriveDistance, -200, 0),
        },

        {
            new AutoCommand(CommandType.SetShooter, 1, 1), 
            new AutoCommand(CommandType.Delay, 1500, 0), //wait while it fires
            new AutoCommand(CommandType.SetShooter, 0, 0),
            new AutoCommand(CommandType.RotateToAngle, 180, 0),
            new AutoCommand(CommandType.SetIntake, 950, 0.5),
            new AutoCommand(CommandType.Delay, 750, 0),
            new AutoCommand(CommandType.DriveDistance, 150, 0),
            new AutoCommand(CommandType.Delay, 300, 0),
            new AutoCommand(CommandType.SetIntake, -82, 0),
        },

        {
            new AutoCommand(CommandType.DriveDistance, 150, 0),
        },
    };

    double dx;
    double dy;

    public AutonomousController(RealTimeDrive RTDrive, Intake intake, Shooter shooter) {
        mRealTimeDrive = RTDrive;
        mIntake = intake;
        mShooter = shooter;
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
        if(takeInputs) {
            autoSequence = (int)SmartDashboard.getNumber("autoSequence", autoSequence);
        }
        return;
    }

    public boolean exitFlag = false;
    public void run() {
        exitFlag = false;
        System.out.println("[AutonomousController] Entered independent service");
        while(!exitFlag) {
            
            SmartDashboard.putNumber("autoState", autoState);
            SmartDashboard.putNumber("commandIndex", pointNum);
            //SmartDashboard.putString("driveMode", mRealTimeDrive.mAutoMode.toString());

            
                
            switch(autoState) {
                case 0:
                    if(pointNum > commands.length) {
                        autoState = -1;
                        break;
                    }
                    switch(commands[autoSequence][pointNum].type) {
                        case DriveToPoint:
                            autoState = 1;
                            break;
                        case SetIntake:
                            //set intake
                            mIntake.setLift(commands[autoSequence][pointNum].aparam);
                            mIntake.setRollers(RollerMode.direct, commands[autoSequence][pointNum].bparam);
                            pointNum++;
                            break;
                        case SetShooter:
                            //set shooter
                            if(commands[autoSequence][pointNum].aparam != 0) {
                                mShooter.setShooterControlMode(ShooterControlMode.velocity);
                            } else {
                                mShooter.setShooterControlMode(ShooterControlMode.none);
                            }
                            if(commands[autoSequence][pointNum].bparam != 0) {
                                autoState = 6;
                            } else {
                                pointNum++;
                            }
                            break;
                        case RotateToAngle:
                            autoState = 2;
                            break;
                        case DriveDistance:
                            autoState = 4;
                            break;
                        case Delay:
                            delayTS = System.nanoTime();
                            currentDelay = (long)commands[autoSequence][pointNum].aparam * 1000000l;
                            autoState = 7;
                            break;
                        default:
                            pointNum++;
                            break;
                    }
                    break;
                case 1: // start driving to point
                    
                    dy = commands[autoSequence][pointNum].point.y - mRealTimeDrive.currentPosition.y;
                    dx = commands[autoSequence][pointNum].point.x - mRealTimeDrive.currentPosition.x;
                    if(Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0)) < 6.0) { //check if we are already close enough
                        pointNum++;
                        autoState = 0;
                    } else {
                        autoState++;
                    }
                    
                    break;
                case 2: //rotate to angle
                    switch(commands[autoSequence][pointNum].type) {
                        case DriveToPoint:
                            dy = commands[autoSequence][pointNum].point.y - mRealTimeDrive.currentPosition.y;
                            dx = commands[autoSequence][pointNum].point.x - mRealTimeDrive.currentPosition.x;
                            mRealTimeDrive.targetAngle = Math.toDegrees(Math.atan2(dy, dx));
                            mRealTimeDrive.setAutoMode(AutoMode.rotate);
                            autoState++;
                            break;
                        case RotateToAngle:
                            System.out.println("[AlignDC] rot to angle");
                            mRealTimeDrive.targetAngle = commands[autoSequence][pointNum].aparam; 
                            mRealTimeDrive.setAutoMode(AutoMode.rotate);
                            autoState++;
                            break;
                        default:
                            autoState = 0;
                            break;

                    }
                    
                    break;
                case 3:
                    if(mRealTimeDrive.autoConditionSatisfied) {
                        switch(commands[autoSequence][pointNum].type) {
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
                    switch(commands[autoSequence][pointNum].type) {
                        case DriveToPoint:
                            dy = commands[autoSequence][pointNum].point.y - mRealTimeDrive.currentPosition.y;
                            dx = commands[autoSequence][pointNum].point.x - mRealTimeDrive.currentPosition.x;
                            mRealTimeDrive.targetDistance = Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0));
                            mRealTimeDrive.setAutoMode(AutoMode.distance);
                            autoState++;
                            break;
                        case DriveDistance:
                            mRealTimeDrive.targetDistance = commands[autoSequence][pointNum].aparam;
                            mRealTimeDrive.setAutoMode(AutoMode.distance);
                            autoState++;
                            break;
                        default:
                            autoState = 0;
                            break;
                    }
                    break;
                case 5:
                    if(mRealTimeDrive.autoConditionSatisfied) {
                        switch(commands[autoSequence][pointNum].type) {
                            case DriveToPoint:
                                dy = commands[autoSequence][pointNum].point.y - mRealTimeDrive.currentPosition.y;
                                dx = commands[autoSequence][pointNum].point.x - mRealTimeDrive.currentPosition.x;
                                if(Math.sqrt(Math.pow(dx, 2.0) + Math.pow(dy, 2.0)) < 15.0) {
                                    pointNum++;
                                    autoState = 0;
                                } else {
                                    autoState = 2;
                                }
                                break;
                            case DriveDistance:
                                pointNum++;
                                autoState = 0;
                                break;
                            default:
                                autoState = 0;
                                break;
                        }
                        mRealTimeDrive.setAutoMode(AutoMode.stop);
                    }
                    break;
                case 6: //await shooter
                    if(mShooter.getAutoConditionSatisfied()) {
                        pointNum++;
                        autoState = 0;
                    } 
                    break;
                case 7:
                    if(System.nanoTime() > delayTS + currentDelay) {
                        pointNum++;
                        autoState = 0;
                    }
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
