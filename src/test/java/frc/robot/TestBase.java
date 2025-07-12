package frc.robot;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;

public class TestBase {
  @BeforeAll
  public static void globalSetup() {
    HAL.initialize(500, 0);
  }
}
