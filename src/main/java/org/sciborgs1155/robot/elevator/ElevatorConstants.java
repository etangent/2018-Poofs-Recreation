package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class ElevatorConstants {
  public static final Measure<Velocity<Distance>> MAX_VELOCITY = MetersPerSecond.of(-1);
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(-1);
  public static final Measure<Distance> MIN_HEIGHT = Meters.of(-1);
  public static final Measure<Distance> MAX_HEIGHT = Meters.of(-1);
  public static final Measure<Distance> POSITION_TOLERANE = Meters.of(-1);
  public static final Measure<Velocity<Distance>> MAX_MANUAL_VELOCITY = MAX_VELOCITY.divide(4);

  public static final double kP = 0;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 0;
  public static final double kA = 0;
  public static final double kG = 0;

  public static class Elevator {
    // reduction
    public static final double HIGH_GEARING = 12.06;
    public static final double LOW_GEARING = 42.44;

    public static final Measure<Mass> CARRIAGE_MASS = Kilograms.of(-1);
    public static final Measure<Distance> DRUM_RADIUS = Meters.of(-1);
  }
}
