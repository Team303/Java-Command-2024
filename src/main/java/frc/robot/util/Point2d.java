package frc.robot.util;

public class Point2d {
    public double x;
    public double y;

    public Point2d(double x, double y){
        this.x=x;
        this.y=y;
    }
    public Point2d(){
        this(0.0,0.0);
    }

    public double dist(Point2d other){
        return Math.hypot(Math.abs(this.x-other.x),Math.abs(this.y-other.y));
    }
    public String toString() {
        return "(" + x + "," + y + ')';
    }
}
