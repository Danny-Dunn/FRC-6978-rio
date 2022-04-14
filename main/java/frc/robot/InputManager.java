package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class InputManager {
    Joystick mStick;

    private enum ControllerType {
        PS5,
        logitechAnalog,
        logitechDigital,
    };

    private ControllerType mControllerType;

    public InputManager(int index) {
        mStick = new Joystick(index);
    }

    boolean map() {
        switch(mStick.getName()) {
            case "Wireless Controller":
                mControllerType = ControllerType.PS5;
                break;
            case "Sony Interactive Entertainment Wireless Controller":
                mControllerType = ControllerType.PS5;
                break;
            case "Logitech Dual Action":
                mControllerType = ControllerType.logitechDigital;
                break;
            case "Controller (Gamepad F310)":
                mControllerType = ControllerType.logitechAnalog;
                break;

            default:
                return false;
        }
        System.out.println("[InputManager] detected " + mControllerType.toString() + " controller");
        return true;
    }

    double getLeftTrigger() {
        switch (mControllerType) {
            case PS5:
                return (mStick.getRawAxis(3) + 1.0) / 2;
            case logitechAnalog:
                return mStick.getRawAxis(3);
            case logitechDigital:
                return mStick.getRawButton(7)? 1.0 : 0.0;
        }
        return 0.0;
    }

    double getRightTrigger() {
        switch (mControllerType) {
            case PS5:
                return (mStick.getRawAxis(4) + 1.0) / 2;
            case logitechAnalog:
                return mStick.getRawAxis(2);
            case logitechDigital:
                return mStick.getRawButton(8)? 1.0 : 0.0; 
                //TODO:verify logitech L/R bumpers & triggers
        }
        return 0.0;
    }

    boolean getLeftTriggerDigital() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawAxis(3) > 0;
            case logitechAnalog:
                return mStick.getRawAxis(3) > 0.5;
            case logitechDigital:
                return mStick.getRawButton(7);
        }
        return false;
    }

    boolean getRightTriggerDigital() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawAxis(4) > 0;
            case logitechAnalog:
                return mStick.getRawAxis(2) > 0.5;
            case logitechDigital:
                return mStick.getRawButton(8); 
                //TODO:verify logitech L/R bumpers & triggers
        }
        return false;
    }

    boolean getRightBumper() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButton(6);
            case logitechAnalog:
                return mStick.getRawButton(6); //TODO:logitech analog right bumper
            case logitechDigital:
                return mStick.getRawButton(6);
        }
        return false;
    }

    boolean getRightBumperPressed() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButtonPressed(6);
            case logitechAnalog:
                return mStick.getRawButtonPressed(6); //TODO:logitech analog right bumper
            case logitechDigital:
                return mStick.getRawButtonPressed(6);
        }
        return false;
    }

    boolean getRightBumperReleased() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButtonReleased(6);
            case logitechAnalog:
                return mStick.getRawButtonReleased(6); //TODO:logitech analog right bumper
            case logitechDigital:
                return mStick.getRawButtonReleased(6);
        }
        return false;
    }


    boolean getLeftBumper() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButton(5);
            case logitechAnalog:
                return mStick.getRawButton(5); //TODO:logitech analog left bumper
            case logitechDigital:
                return mStick.getRawButton(5);
        }
        return false;
    }

    boolean getNorthButton() {
        return mStick.getRawButton(4);
    }

    boolean getSouthButton() {
        return mStick.getRawButton(2);
    }

    boolean getEastButton() {
        return mStick.getRawButton(3);
    }

    boolean getWestButton() {
        return mStick.getRawButton(1);
    }

    boolean getNorthButtonPressed() {
        return mStick.getRawButtonPressed(4);
    }

    boolean getSouthButtonPressed() {
        return mStick.getRawButtonPressed(2);
    }

    boolean getEastButtonPressed() {
        return mStick.getRawButtonPressed(3);
    }

    boolean getWestButtonPressed() {
        return mStick.getRawButtonPressed(1);
    }

    boolean getNorthButtonReleased() {
        return mStick.getRawButtonReleased(4);
    }

    boolean getSouthButtonReleased() {
        return mStick.getRawButtonReleased(2);
    }

    boolean getEastButtonReleased() {
        return mStick.getRawButtonReleased(3);
    }

    boolean getWestButtonReleased() {
        return mStick.getRawButtonReleased(1);
    }

    boolean getLeftSystemButton() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButton(9);
            case logitechAnalog:
                return mStick.getRawButton(7); 
            case logitechDigital:
                return mStick.getRawButton(9);
        }
        return false;
    }
    
    boolean getRightSystemButton() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButton(10);
            case logitechAnalog:
                return mStick.getRawButton(8); 
            case logitechDigital:
                return mStick.getRawButton(10);
        }
        return false;
    }

    boolean getLeftSystemButtonPressed() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButtonPressed(9);
            case logitechAnalog:
                return mStick.getRawButtonPressed(7); 
            case logitechDigital:
                return mStick.getRawButtonPressed(9);
        }
        return false;
    }

    boolean getLeftSystemButtonReleased() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButtonReleased(9);
            case logitechAnalog:
                return mStick.getRawButtonReleased(7); 
            case logitechDigital:
                return mStick.getRawButtonReleased(9);
        }
        return false;
    }
    
    boolean getRightSystemButtonPressed() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawButtonPressed(10);
            case logitechAnalog:
                return mStick.getRawButtonPressed(8); 
            case logitechDigital:
                return mStick.getRawButtonPressed(10);
        }
        return false;
    }

    double getLeftStickX() {
        return mStick.getX();
    }

    double getLeftStickY() {
        return mStick.getX();
    }

    double getRightStickX() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawAxis(2);
            case logitechAnalog:
                return mStick.getRawAxis(3);
            case logitechDigital:
                return mStick.getRawAxis(2); 
                //TODO:verify logitech L/R bumpers & triggers
        }
        return 0;
    }

    double getRightStickY() {
        switch (mControllerType) {
            case PS5:
                return mStick.getRawAxis(3);
            case logitechAnalog:
                return mStick.getRawAxis(4);
            case logitechDigital:
                return mStick.getRawAxis(3); 
                //TODO:verify logitech L/R bumpers & triggers
        }
        return 0;
    }

    int getPOV() {
        return mStick.getPOV();
    }
}
