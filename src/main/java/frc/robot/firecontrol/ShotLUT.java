package frc.robot.firecontrol;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

// distance-keyed lookup table for RPM, hood angle, and TOF
public class ShotLUT {

  private final InterpolatingTreeMap<Double, ShotParameters> map;
  private int entryCount = 0;

  public ShotLUT() {
    map = new InterpolatingTreeMap<>(
        InverseInterpolator.forDouble(), ShotParameters.interpolator());
  }

  public void put(double distanceM, ShotParameters params) {
    map.put(distanceM, params);
    entryCount++;
  }

  // convenience: insert from individual fields
  public void put(double distanceM, double rpm, double angleDeg, double tofSec) {
    put(distanceM, new ShotParameters(rpm, angleDeg, tofSec));
  }

  public ShotParameters get(double distanceM) {
    ShotParameters result = map.get(distanceM);
    return result != null ? result : ShotParameters.ZERO;
  }

  public double getRPM(double distanceM) { return get(distanceM).rpm(); }
  public double getAngle(double distanceM) { return get(distanceM).angleDeg(); }
  public double getTOF(double distanceM) { return get(distanceM).tofSec(); }

  public void clear() {
    map.clear();
    entryCount = 0;
  }

  public int size() { return entryCount; }
}
