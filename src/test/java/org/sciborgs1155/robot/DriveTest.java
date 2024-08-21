package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.*;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.CsvSource;
import org.sciborgs1155.lib.InputStream;
import org.sciborgs1155.robot.drive.Drive;
import org.sciborgs1155.robot.drive.SimDrive;

public class DriveTest {
  private Drive drive;
  private final double DELTA = 0.1;

  @BeforeEach
  void setup() {
    setupHAL();
    drive = new Drive(new SimDrive());
  }

  @AfterEach
  void destroy() throws Exception {
    reset(drive);
  }

  @ParameterizedTest
  @CsvSource({"2, 2", "1, 1", "-1, -1", "-2, -2", "2, -2", "-1, 2"})
  void reachesVelocity(double leftSpeed, double rightSpeed) {
    run(drive.drive(InputStream.of(() -> leftSpeed), InputStream.of(() -> rightSpeed)));
    fastForward();

    DifferentialDriveWheelSpeeds speeds = drive.wheelSpeeds();

    assertEquals(leftSpeed, speeds.leftMetersPerSecond, DELTA);
    assertEquals(rightSpeed, speeds.rightMetersPerSecond, DELTA);
  }
}
