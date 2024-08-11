package org.sciborgs1155.robot.wrist;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/*
 * some constants are made up for simulation purposes
 * if they are, they are preceded by //
 */
public class WristConstants {
  //
  public static final Measure<Velocity<Angle>> MAX_VELOCITY = RadiansPerSecond.of(3);
  //
  public static final Measure<Velocity<Velocity<Angle>>> MAX_ACCELERATION =
      RadiansPerSecond.per(Second).of(6);
  public static final Measure<Angle> MAX_ANGLE = Radians.of(Math.PI);
  public static final Measure<Angle> MIN_ANGLE = Radians.of(0);
  public static final Measure<Angle> POSITION_TOLERANCE = Radians.of(.01);

  public static final double kP = 10;
  public static final double kI = 0;
  public static final double kD = .5;
  public static final double kS = 0;
  public static final double kG = 1.835;
  public static final double kV = 1;
  public static final double kA = 0;

  public static class Pivot {
    // reduction
    public static final double GEARING = 150;

    //
    public static final Measure<Distance> LENGTH = Meters.of(.5);
    public static final Measure<Mass> MASS = Kilograms.of(8);
  }
}
