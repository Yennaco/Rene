package org.firstinspires.ftc.teamcode;

public class PIDFController {

    private double kP;
    private double kI;
    private double kD;
    private double kAcceleration;
    private double kJerk;

    private double I_Sum;

    private double target;

    private double tolerance;

    private double last;

    private boolean isFirstTime;
    private double timesInTolerance;

    public PIDFController(double kp, double ki, double kd, double kAcceleration, double kJerk, double tolerance)
    {
        kP = kp;
        kI = ki;
        kD = kd;
        this.kAcceleration = kAcceleration;
        this.kJerk = kJerk;
        I_Sum = 0;
        target = 0;
        this.tolerance = tolerance;
        last = 0;
        isFirstTime = true;
    }

    public double getOutput(double current)
    {
        if(isFirstTime)
        {
            last = current; //  This just checks to see if the robot is in the first cycle so that it can make the last val equal to the current so that the d term is 0 at the start
            isFirstTime = false;
        }

        double error = current-target;

        if (((last - target) * (current - target) < 0) || (Math.abs(error) < tolerance))
        {
            I_Sum = 0;
        }

        I_Sum += error;

        double pTerm = error * kP;
        double dTerm = (current - last) * kD;
        double iTerm = I_Sum * kI;

        last = current;

        if (Math.abs(error) < tolerance)
        {
            timesInTolerance++;
        }
        else
        {
            timesInTolerance = 0;
        }

        return pTerm + dTerm + iTerm;
    }

    public boolean isDone()
    {
        return timesInTolerance > 5;
    }



}
