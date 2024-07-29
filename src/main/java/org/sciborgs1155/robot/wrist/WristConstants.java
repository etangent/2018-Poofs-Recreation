package org.sciborgs1155.robot.wrist;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class WristConstants {
  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(-1);
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCELERATION =
      RadiansPerSecond.per(Second).of(-1);
  public static final Measure<Angle> MAX_ANGLE = Radians.of(-1);
  public static final Measure<Angle> MIN_ANGLE = Radians.of(-1);

  public static class Pivot {
    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double kS = 0;
    public static final double kG = 0;
    public static final double kV = 0;
    public static final double kA = 0;
    // reduction
    public static final double GEARING = 150;
  }
}
