package org.sciborgs1155.robot.drive;

import edu.wpi.first.math.geometry.Rotation2d;

public interface DriveIO {
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

  public Rotation2d getHeading();

  /**
   * @param high if true, shifts into high gear; false shifts to low gear
   */
  public void shiftGears(boolean high);
}
