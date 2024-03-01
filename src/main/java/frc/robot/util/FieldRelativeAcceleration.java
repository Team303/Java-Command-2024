package frc.robot.util;

public class FieldRelativeAcceleration {
    public double ax;
    public double ay;
    public double alpha;

    public FieldRelativeAcceleration(double ax, double ay, double alpha) {
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    public FieldRelativeAcceleration(FieldRelativeSpeeds newSpeeds, FieldRelativeSpeeds oldSpeeds, double time) {
        this.ax = (newSpeeds.vx - oldSpeeds.vx) / time;
        this.ay = (newSpeeds.vy - oldSpeeds.vy) / time;
        this.alpha = (newSpeeds.omega - oldSpeeds.omega) / time;

        if (Math.abs(this.ax) > 6.0) {
            this.ax = 6.0 * Math.signum(this.ax);
        }
        if (Math.abs(this.ay) > 6.0) {
            this.ay = 6.0 * Math.signum(this.ay);
        }
        if (Math.abs(this.alpha) > 4 * Math.PI) {
            this.alpha = 4 * Math.PI * Math.signum(this.alpha);
        }
    }

    public FieldRelativeAcceleration() {
        this(0.0, 0.0, 0.0);
    }

}
