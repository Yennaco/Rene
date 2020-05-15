package org.firstinspires.ftc.teamcode.Entities;

public class SimpleEntity implements Entity {
    public double time = 0;
    public double val = 0;
    public double velocity = 0;
    public double acceleration = 0;

    public SimpleEntity()
    {
        time = 0;
        val = 0;
        velocity = 0;
        acceleration = 0;
    }

    public SimpleEntity(double time, double val, double velocity, double acceleration)
    {
        this.time = time;
        this.val = val;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public double getTime()
    {
        return time;
    }

    public double getVal()
    {
        return val;
    }

    public double getVelocity()
    {
        return velocity;
    }

    public double getAcceleration()
    {
        return acceleration;
    }
}
