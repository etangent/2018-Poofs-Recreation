package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;

public interface DriveIO extends AutoCloseable {
  public void setVoltages(double leftVoltage, double rightVoltage);

  /*
   * Meters
   */
  public double getLeftPosition();

  public double getRightPosition();

  /*
   * Meters per second
   */
  public double getRightVelocity();

  public double getLeftVelocity();

  public Measure<Voltage> getLeftVoltage();

  public Measure<Voltage> getRightVoltage();

  public Rotation2d getHeading();
}
