package org.sciborgs1155.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.TestingUtil.fastForward;
import static org.sciborgs1155.lib.TestingUtil.reset;
import static org.sciborgs1155.lib.TestingUtil.run;
import static org.sciborgs1155.lib.TestingUtil.setupHAL;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.ValueSource;
import org.sciborgs1155.robot.wrist.Wrist;

public class WristTest {
  private Wrist wrist;
  private final double DELTA = 0.05;

  @BeforeEach
  void setup() {
    setupHAL();
    wrist = Wrist.create();
  }

  @AfterEach
  void destroy() throws Exception {
    reset(wrist);
  }

  @ParameterizedTest
  @ValueSource(doubles = {Math.PI, Math.PI / 2, Math.PI / 4})
  void reachesAngularPosition(double angle) {
    run(wrist.goTo(() -> angle));
    fastForward();
    assertEquals(angle, wrist.measurement(), DELTA);
  }
}
