/*

TurretRotationalControl -- represents the code needed to control the robot's
physical turret system.

The turret rotates around a central axis where 0 degrees is facing straight
forward on the robot. The methods operate on the basis that angles are set and
reported as between -180 and +180.

*/

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.can.CANTimeoutException;


//------------------------------------------------------------------------------
public class TurretRotationControl {
//------------------------------------------------------------------------------
    final double DEGREES_PER_REVOLUTION = 24.33;   // degrees (measured)
    final double ACCEPT_POSITION_RANGE  = 0.08;    // turn
    final double MAX_RANGE_OF_POT       = 3.5;    // turns allowed by hardware (calculated for aprox. 85 degrees)

    private CANJaguar jag;

    private double MIN_POS;
    private double MAX_POS;
    private double MIN_ANGLE;
    private double MAX_ANGLE;

    private double home;
    private double currentPos;
    private double currentAngle;

    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    public TurretRotationControl() {
    //- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
        try {
            jag = new CANJaguar(Wiring.TURRET_ROT_JAGUAR_ID);
            jag.configMaxOutputVoltage(7.1);

            jag.changeControlMode(CANJaguar.ControlMode.kPosition);
            jag.configPotentiometerTurns(10);
            jag.setPositionReference(CANJaguar.PositionReference.kPotentiometer);

            jag.setPID(300, 0.0001, 0.0);
            jag.enableControl();

            currentPos = home = jag.getPosition();
            currentAngle = rot2Angle(currentPos);

            MIN_POS = home - MAX_RANGE_OF_POT/2;
            MAX_POS = home + MAX_RANGE_OF_POT/2;

            MIN_ANGLE = -rot2Angle(MIN_POS);
            MAX_ANGLE = -rot2Angle(MAX_POS);

          //  System.out.println("home: "+ home + " " + MIN_POS + " " + MAX_POS + " " + MIN_ANGLE + " " + MAX_ANGLE);

        } catch (CANTimeoutException ex) {
            ex.printStackTrace();
        }
    }

    /**
     * gets the current angle of the turret
     *
     * @return angle in range -180 <= angle <= 180
     */
    public double getAngle() throws CANTimeoutException {
        return (home - jag.getPosition()) * DEGREES_PER_REVOLUTION;
    }

    /**
     * determine if turret is at it's MIN or MAX values (w/ an adjustment)
     *
     * @param rangeAdjust shrinks the min/max range by specified amount
     * @return boolean representing this check
     */
    public boolean atMinMaxStop(double rangeAdjust) throws CANTimeoutException {
        double cta = getAngle();
        return !(MIN_ANGLE + rangeAdjust <= cta && cta <= MAX_ANGLE - rangeAdjust);
    }

    /**
     *  set the turret angle to an absolute angle specified in degrees. It will
     *  get capped at the max or min turret rotation angle.
     *
     * @param angleDegrees angle in degrees between -180 <= angle <= 180
     */
    public void setAngle(double angleDegrees, int delay) throws CANTimeoutException {
        currentPos = home - angleDegrees / DEGREES_PER_REVOLUTION;
        if (currentPos > MAX_POS)
            currentPos = MAX_POS;
        else if (currentPos < MIN_POS)
            currentPos = MIN_POS;

        currentAngle = rot2Angle(currentPos);
        jag.setX(currentPos);
        sleep(delay);
    }

    /**
     *  set the turret angle specified in degrees relative to it's current position.
     *  It will get capped at the max or min turret rotation angle.
     *
     * @param angleDegrees angle in degrees between -180 <= angle <= 180
     */
    public void setAngleRelative(double angleDegrees) throws CANTimeoutException {
        setAngle(currentAngle + angleDegrees, 100);
       // System.out.println("setAngleRelative (current angle): " + currentAngle);
    }

     /**
     *  set the turret angle specified in degrees relative to it's current position.
     *  It will get capped at the max or min turret rotation angle.
     *
     * @param angleDegrees angle in degrees between -180 <= angle <= 180
     */
    public void setAngleRelative(double angleDegrees, int msPause) throws CANTimeoutException {
        setAngle(currentAngle + angleDegrees, msPause);
      //  System.out.println("setAngleRelative (current angle): " + currentAngle);
    }

    /**
     * set the angle to maximum value that the turret can go
     * this can be adjusted.
     *
     * @param adjustAngle an angle in degrees that offsets from the actual max
     */
    public void setAngleMax(double adjustAngle) throws CANTimeoutException {
        setAngle(MAX_ANGLE - adjustAngle, 100);
    }

    /**
     * set the angle to minumum value that the turret can go
     * this can be adjusted.
     *
     * @param adjustAngle an angle in degrees that offsets from the actual min
     */
    public void setAngleMin(double adjustAngle) throws CANTimeoutException {
        setAngle(MIN_ANGLE + adjustAngle, 100);
    }

    /**
     * send the turret to the home position '0 degrees'
     */
    public void goHome() throws CANTimeoutException{
        setAngle(0, 1000);
        System.out.println("$$$$$$$ GO HOME !!!! $$$$$$$\n");
        System.out.println("Home: " + (currentPos - home));
    }


    protected double rot2Angle(double rot) {
        return (home - rot) * DEGREES_PER_REVOLUTION;
    }

    protected double angle2Rot(double deg) {
        return home - deg / DEGREES_PER_REVOLUTION;
    }


    public void sleep(int ms) {
        try { Thread.sleep(ms);
        } catch (InterruptedException ex) {}
    }

    public void test() throws CANTimeoutException {
        setAngle(0, 100);
        for (int i=0; i<30; i++) {
            // setAngleRelative(0.3);
            // setAngle(0.3 * i);
            sleep(1000);
         //   System.out.println(getAngle());
        }
    }
}
