package frc.robot.driver;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.MotorDriver;

public class SparkMaxDriver implements MotorDriver{
    private CANSparkMax mSparkMax;
    private RelativeEncoder mEncoder;
    
    public SparkMaxDriver(int id) {
        mSparkMax = new CANSparkMax(id, MotorType.kBrushless);
        mEncoder = mSparkMax.getEncoder();
    }

    public void set(double demand) {
        mSparkMax.set(demand);
    }

    public void disable() {
        mSparkMax.disable();
    }

    public double getInputVoltage() {
        return mSparkMax.getBusVoltage();
    }

    public double getOutputVoltage() {
        return mSparkMax.getAppliedOutput() * getInputVoltage();
    }

    public double getOutputCurrent() {
        return mSparkMax.getOutputCurrent();
    }

    public double getOutputPower() {
        return getOutputVoltage() / getOutputCurrent();
    }

    public double getAppliedOutput() {
        return mSparkMax.getAppliedOutput();
    }

    public void setEncoderInversion(boolean invert) {
        mEncoder.setInverted(invert);
    }

    public void setDriverInversion(boolean invert) {
        mSparkMax.setInverted(invert);
    }

    public double getRelativePosition() {
        return mEncoder.getPosition();
    }

    public void setRelativePosition(double position) {
        mEncoder.setPosition(position);
    }

    public double getVelocity() {
        return mEncoder.getVelocity();
    }
}
