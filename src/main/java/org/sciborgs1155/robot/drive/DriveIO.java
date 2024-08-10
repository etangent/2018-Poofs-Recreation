package org.sciborgs1155.robot.drive;

public interface DriveIO {
    public void setVoltages(double leftVoltage, double rightVoltage);

    /*
     * Meters per second
     */
    public double getRightVelocity();

    public double getLeftVelocity();

    /**
     * @param high if true, shifts into high gear; false shifts to low gear
     */
    public void shiftGears(boolean high);
}
