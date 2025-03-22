package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import javax.swing.JFrame;

public class KeyboardInput extends SubsystemBase {
  private String lastKeyPressed = "";

  public KeyboardInput() {
    // Create a JFrame to register the KeyListener
    JFrame frame = new JFrame("Key Controller");
    frame.addKeyListener(
        new KeyListener() {
          @Override
          public void keyPressed(KeyEvent e) {
            lastKeyPressed = KeyEvent.getKeyText(e.getKeyCode());
          }

          @Override
          public void keyReleased(KeyEvent e) {
            // Clear the key state when released
            lastKeyPressed = "";
          }

          @Override
          public void keyTyped(KeyEvent e) {
            // Optional: Handle key typed events
          }
        });
    frame.setSize(200, 200); // Dummy size
    frame.setVisible(true);
  }

  public String getLastKeyPressed() {
    return lastKeyPressed;
  }
}
