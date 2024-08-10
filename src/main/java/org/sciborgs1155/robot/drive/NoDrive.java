package org.sciborgs1155.robot.drive;

public class NoDrive implements DriveIO {
    @Override
    public void setVoltages(double leftVoltage, double rightVoltage) {}

    @Override
    public double getLeftVelocity() {
        return 0;
    }

    @Override
    public double getRightVelocity() {
        return 0;
    }

    @Override
    public void shiftGears(boolean high) {}
}
