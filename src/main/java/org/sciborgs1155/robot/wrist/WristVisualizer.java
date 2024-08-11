package org.sciborgs1155.robot.wrist;

import static org.sciborgs1155.robot.wrist.WristConstants.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class WristVisualizer implements Sendable {
  private final Mechanism2d mech = new Mechanism2d(30, 30);
  private final MechanismLigament2d wrist;

  public WristVisualizer(Color8Bit color) {
    MechanismRoot2d chassis = mech.getRoot("Chassis", 10, 10);
    wrist = chassis.append(new MechanismLigament2d("wrist", 5, 0, 4, color));
  }

  public void setAngle(double radians) {
    wrist.setAngle(Units.radiansToDegrees(radians));
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    mech.initSendable(builder);
  }
}
