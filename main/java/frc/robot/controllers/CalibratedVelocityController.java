package frc.robot.controllers;

import java.sql.Driver;

import frc.robot.MotorDriver;

public class CalibratedVelocityController {
    //Takes a velocity demand, combines a voltage calibrated demand and a
    // PID demand to control the velocity of a motor
    
    
    private MotorDriver mDriver;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double ticksPerVolt;

    private double velocityDemand;

    private double integral;
    private long previousTS;
    private double previousError;

    public CalibratedVelocityController(MotorDriver driver, double ticksPerVolt) {
        mDriver = driver;
        this.ticksPerVolt = ticksPerVolt;
    }

    public void setPIDConstants(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double getDesiredVoltage(double velocity) {
        return velocity / ticksPerVolt;
    }

    public double getCalibratedDemand(double velocity) {
        return getDesiredVoltage(velocity) / mDriver.getInputVoltage();
    }

    public void setVelocityDemand(double demand) {
        velocityDemand = demand;
    }

    public double getPIDDemand() {
        double error, demand, derivative, deltaT;

        deltaT = System.nanoTime() - previousTS;
        previousTS = System.nanoTime();
        
        error = velocityDemand - mDriver.getVelocity();

        integral += (deltaT * error) / 10000000000d;

        derivative = (error - previousError) / deltaT;
        previousError = error;

        demand = (error * kP);
        demand += (integral * kI);
        demand += (derivative * kD);

        return demand;
    }

    //should be called before first run
    public void prepare() {
        previousTS = System.nanoTime();
        integral = 0;
    }

    public void refresh() {
        mDriver.set(getPIDDemand() + getCalibratedDemand(velocityDemand));
    }
}
