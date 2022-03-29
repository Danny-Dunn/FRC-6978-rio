package frc.robot;

import edu.wpi.first.wpilibj.drive.Vector2d;

public class AutoCommand {
    enum CommandType {
        RotateToAngle,
        DriveToPoint,
        SetIntake,
        DriveDistance,
        SetShooter,
        RunLoader,
        Delay
    };

    CommandType type;

    Vector2d point;
    double angle;
    double distance;

    double aparam;
    double bparam;

    //drive to coordinate
    public AutoCommand(double X, double Y) {
        type = CommandType.DriveToPoint;
        point = new Vector2d(X, Y);
    }

    //drive distance
    public AutoCommand(double distance) {
        type = CommandType.DriveDistance;
        this.distance = distance;
    }

    //other commands
    public AutoCommand(CommandType type, double aparam, double bparam) {
        this.type = type;
        this.aparam = aparam;
        this.bparam = bparam;
    }
}
