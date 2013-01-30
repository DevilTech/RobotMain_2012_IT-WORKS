/*
 * Provides wrapper functions around standard gyro class that returns more useful
 * values.
 */
package edu.wpi.first.wpilibj.templates;

public class Gyro extends edu.wpi.first.wpilibj.Gyro {

    public Gyro(int analogInput) {
        super(analogInput);
    }

    public double getAngle360() {
        double angle = super.getAngle() % 360;
        if (angle < 0)
            angle += 360;
        return angle;
    }

    public double getAngle180() {
        double angle = super.getAngle() % 360;
        if (angle < 0)
            angle += 360;
        if (angle > 180)
            angle -= 360;
        return angle;
    }
}
