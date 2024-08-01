package org.sciborgs1155.robot.wrist;

import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;
import static org.sciborgs1155.robot.wrist.WristConstants.MAX_ANGLE;
import static org.sciborgs1155.robot.wrist.WristConstants.MIN_ANGLE;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.sciborgs1155.robot.Constants;
import org.sciborgs1155.robot.wrist.WristConstants.Pivot;

public class SimWrist implements WristIO {
  private final SingleJointedArmSim pivot;

  public SimWrist() {
    pivot =
        new SingleJointedArmSim(
            DCMotor.getVex775Pro(1),
            Pivot.GEARING,
            SingleJointedArmSim.estimateMOI(Pivot.LENGTH.in(Meters), Pivot.MASS.in(Kilograms)),
            Pivot.LENGTH.in(Meters),
            MIN_ANGLE.in(Radians),
            MAX_ANGLE.in(Radians),
            true,
            MIN_ANGLE.in(Radians));
  }

  @Override
  public void setVoltage(double voltage) {
    pivot.setInputVoltage(voltage);
    pivot.update(Constants.PERIOD.in(Seconds));
  }

  @Override
  public double getPosition() {
    return pivot.getAngleRads();
  }

  @Override
  public double getVelocity() {
    return pivot.getVelocityRadPerSec();
  }

  @Override
  public boolean atLimitSwitch() {
    return false;
  }

  // obviously pointless; limit switch is never true here
  @Override
  public void zeroEncoders() {}
}
