package org.sciborgs1155.robot.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

/*
 * some constants are made up here because I need them to simulate
 * if they are made up, they will be preceded by a //
 */
public class ElevatorConstants {
  //
  public static final Measure<Velocity<Distance>> MAX_VELOCITY = MetersPerSecond.of(3);
  //
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCELERATION =
      MetersPerSecondPerSecond.of(6);
  //
  public static final Measure<Distance> HEIGHT_FROM_GROUND = Meters.of(.3);
  //
  // in addition to being fake, this is relative to the bottom of travel
  public static final Measure<Distance> MAX_HEIGHT = Meters.of(2);
  //
  public static final Measure<Distance> POSITION_TOLERANCE = Meters.of(.1);
  public static final Measure<Velocity<Distance>> MAX_MANUAL_VELOCITY = MAX_VELOCITY.divide(2);

  public static final double kP = 10;
  public static final double kI = 0;
  public static final double kD = .1;
  public static final double kS = 0;
  public static final double kV = 2;
  public static final double kA = 0;
  public static final double kG = 1.75;

  public static class Elevator {
    // reduction
    public static final double HIGH_GEARING = 12.06;
    public static final double LOW_GEARING = 42.44;

    //
    public static final Measure<Mass> CARRIAGE_MASS = Kilograms.of(20);
    //
    public static final Measure<Distance> DRUM_RADIUS = Inches.of(1);
  }
}
