package frc.robot.firecontrol;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolator;

// RPM, hood angle, and TOF at a single distance. All three interpolate linearly.
public record ShotParameters(double rpm, double angleDeg, double tofSec) {

  public static final ShotParameters ZERO = new ShotParameters(0, 0, 0);

  // linear interpolator for WPILib InterpolatingTreeMap
  public static Interpolator<ShotParameters> interpolator() {
    return (start, end, t) ->
        new ShotParameters(
            MathUtil.interpolate(start.rpm, end.rpm, t),
            MathUtil.interpolate(start.angleDeg, end.angleDeg, t),
            MathUtil.interpolate(start.tofSec, end.tofSec, t));
  }
}
