package frc.robot;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.AnalogGyro;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;
import frc.robot.WheelCommand;

//3863 implementation of https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf
//special thanks to Solomon and Prateek
//thanks to Jaci for her work on Pathfinder!

public class RamseteFollower{

    TalonSRX left, right;
    double k_1, k_2, k_3, v, w, v_d, w_d, theta_d, x_d, y_d;
    double wheelBase;
    AnalogGyro gyro;
    Trajectory path;

    volatile double x, y, theta;

    static final double b = 0;
    static final double zeta = 0;
    static final double wheelDiameter = 4;

    int index, numSegments;
    //where v = linear velocity (feet/s) and w = angular velocity (rad/s)
    public RamseteFollower(TalonSRX left, TalonSRX right, double wheelBase, Trajectory path, AnalogGyro gyro){
        this.left = left;
        this.right = right;
        this.gyro = gyro;
        this.wheelBase = wheelBase;
        k_2 = b;
        index = 0;
        numSegments = path.length()-1;
        startOdometry();
    }

    public WheelCommand getNextWheelCommand(){
        double left = 0;
        double right = 0;
        if(index == numSegments){
            return new WheelCommand(left, right);
        }
        Segment current = path.get(index);
        calcVel(current.x, current.y, Math.toRadians(current.heading), current.velocity, Math.toRadians(current.heading)/current.dt);
        calcAngleVel(current.x, current.y, Math.toRadians(current.heading), current.velocity, Math.toRadians(current.heading)/current.dt);


        left = (-wheelBase*w)/2 + v;
        right = (+wheelBase*w)/2 + v;

        return new WheelCommand(left, right);
    }

    public void calcVel(double x_d, double y_d, double theta_d, double v_d, double w_d){
        calcK(v_d, w_d);
        double calcV = v_d * Math.cos(theta_d - Math.toRadians(gyro.getAngle())) + k_1 * (Math.cos(theta)*(x_d - x) + Math.sin(theta)*(y_d-y));
        v = calcV;
    }

    public void calcAngleVel(double x_d, double y_d, double theta_d, double v_d, double w_d){
        calcK(v_d, w_d);
        double calcW = w_d + k_2 * v_d * (Math.sin(theta_d - theta) / (theta_d - theta)) * (Math.cos(y_d - y) - Math.sin(theta)*(x_d - x)) + k_3 * (theta_d - theta);
        w = calcW;
    }

    public void calcK(double v_d, double w_d){
        k_1 = 2 * zeta * Math.sqrt(Math.pow(w_d,2)+ k_2*Math.pow(v_d, 2));
        k_3 = k_1;
    }

    public void startOdometry(){
        new Thread (()->{
                while(true){
                    x += Math.cos(Math.toRadians(gyro.getAngle())) * talonNativeToFPS(((left.getSelectedSensorVelocity(0) + right.getSelectedSensorVelocity(0)) / 2));
                    y += Math.sin(Math.toRadians(gyro.getAngle())) * talonNativeToFPS(((left.getSelectedSensorVelocity(0) + right.getSelectedSensorVelocity(0)) / 2));
                    theta = Math.toRadians(gyro.getAngle());
                }
        }).start();
    }

    public double talonNativeToFPS(double something){
        return (Math.PI * 10 * wheelDiameter * something) / (4 * 128);
    }

    public boolean isFinished(){
        return index == numSegments;
    }
}