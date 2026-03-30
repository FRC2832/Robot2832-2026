package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class LookupTable {

    public record Result(AngularVelocity shooterSpeed, double hoodServoSetting) {
    }

    Distance[] distances;
    AngularVelocity[] shooterSpeeds;
    double[] hoodSettings;

    /**
     * Create a lookup table based on the specified mapping.<br>
     * <br>
     * 
     * The two arrays passed to the function must be the same size.
     * 
     * @param distances     A list of distances mapping to the corresponding outputs.
     *                      Lookup will interpolate between outputs based on the
     *                      known distance/speed and distance/hood setting mappings
     * @param shooterSpeeds A list of speed values mapped to by the distances list.
     *                      Must be the same length as inputs
     * @param hoodSettings  A list of hood servo values mapped to by the distances
     *                      list. Must be the same length as inputs
     */
    public LookupTable(Distance[] distances, AngularVelocity[] shooterSpeeds, double[] hoodSettings) {
        this.distances = distances;
        this.shooterSpeeds = shooterSpeeds;
        this.hoodSettings = hoodSettings;
    }

    /**
     * Maps the specified distance to a shooter speed and hood servo setting based
     * on the lookup table
     * 
     * @param distanceToTarget the distance to the target to interpolate
     * @return An array containing the speed of the shooter and the setting for the
     *         hood servo, in that order
     */
    public Result lookup(Distance distanceToTarget) {
        int targetDex = -1;
        for (int i = distances.length - 1; i >= 0; i--) {
            if (distances[i].lt(distanceToTarget)) {
                targetDex = i;
                break;
            }
        }
        if (targetDex == -1)
            return new Result(shooterSpeeds[0], hoodSettings[0]); // need more data
        if (targetDex == distances.length - 1)
            return new Result(shooterSpeeds[targetDex], hoodSettings[targetDex]); // need more
                                                                                                         // data
        double offset = MathUtil.inverseInterpolate(distances[targetDex].in(Meters), distances[targetDex + 1].in(Meters), distanceToTarget.in(Meters));
        double shooterSpeed = MathUtil.interpolate(shooterSpeeds[targetDex].in(RotationsPerSecond), shooterSpeeds[targetDex + 1].in(RotationsPerSecond), offset);
        double hoodSetting = MathUtil.interpolate(hoodSettings[targetDex], hoodSettings[targetDex + 1], offset);
        return new Result(RotationsPerSecond.of(shooterSpeed), hoodSetting);
    }
}