package frc.robot;

public class WheelCommand{
    double left, right;
    public WheelCommand(double left, double right){
        this.left = left;
        this.right = right;
    }

    public double getLeft(){return left;}

    public double getRight(){return right;}

}