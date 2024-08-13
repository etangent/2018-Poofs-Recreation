package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.TestingUtil.reset;

import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.forklift.Forklift;

public class ForkliftTest {
  @Test
  void initialize() throws Exception {
    Forklift.create().close();
    reset();
  }
}
