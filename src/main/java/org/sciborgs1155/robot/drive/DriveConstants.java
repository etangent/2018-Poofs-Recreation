package org.sciborgs1155.robot.drive;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N7;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Mass;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;

public class DriveConstants {
  public static final Pose2d INITIAL_POSE = new Pose2d(.5, 7, Rotation2d.fromRadians(Math.PI));
  public static final Measure<Velocity<Distance>> MAX_SPEED = MetersPerSecond.of(.5);
  public static final Measure<Velocity<Velocity<Distance>>> MAX_ACCEL =
      MetersPerSecondPerSecond.of(5);
  public static final Measure<Velocity<Distance>> VELOCITY_TOLERANCE = MetersPerSecond.of(.1);

  public static final double kP = 1.6988;
  public static final double kI = 0;
  public static final double kD = 0;
  public static final double kS = 0;
  public static final double kV = 1.3679;
  public static final double kA = 0.4927;

  public static class Drivetrain {
    public static final double HIGH_GEARING = 7.33;
    public static final double LOW_GEARING = 5.56;
    // made up
    public static final double MOI = 4;
    public static final Measure<Mass> DRIVE_MASS = Kilograms.of(25);
    public static final Measure<Distance> WHEEL_RADIUS = Inches.of(4);
    public static final Measure<Distance> TRACK_WIDTH = Inches.of(17.75);
    public static final Matrix<N7, N1> STD_DEVS = VecBuilder.fill(0, 0, 0, 0, 0, 0, 0);
  }
}
