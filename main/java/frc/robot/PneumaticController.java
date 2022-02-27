package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PneumaticController implements Runnable {
    Joystick driveStick; //joystick(only used for button inputs)
    Compressor compressor;
    Solenoid solenoid;
    public boolean exitFlag;
    
    boolean compressorState = false;
    boolean solenoidState = true;

    public PneumaticController(Joystick driverStick) {
        this.driveStick = driverStick;
    }

    public void init() {
        compressor = new Compressor(PneumaticsModuleType.CTREPCM);
        compressor.disable();
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        compressor.enableDigital();
    }

    public void run() {
        exitFlag = false;
        while (!exitFlag) {
            if (driveStick.getRawButton(1)) compressorState = !compressorState;
            if (driveStick.getRawButton(2)) solenoidState = !solenoidState;

            compressor.disable();
            solenoid.set(solenoidState);

            SmartDashboard.putBoolean("compressor", compressorState);
            SmartDashboard.putBoolean("solenoid", solenoidState);
        }
        compressor.disable();
        solenoid.set(false);
    }
}