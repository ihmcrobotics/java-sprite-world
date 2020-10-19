package us.ihmc.javaSpriteWorld;

import java.awt.BorderLayout;
import java.awt.Container;

import javax.swing.JButton;
import javax.swing.JFrame;

import javafx.scene.control.Button;

public class SpriteWorldViewerUsingSwing implements SpriteWorldViewer
{
   private final String name;

   private JFrame jFrame;
   private SpriteWorldPanel spriteWorldPanel;

   private int preferredWidth = 1000, preferredHeight = 1000;
   private boolean resizable = false;

   private SpriteWorld spriteWorld;

   private int startupLocationOnScreenX = -1;
   private int startupLocationOnScreenY = -1;

   public SpriteWorldViewerUsingSwing(String name)
   {
      this.name = name;
   }

   /* (non-Javadoc)
    * @see com.mindbook.spriteEngine.SpriteWorldViewer#setPreferredSizeInPixels(int, int)
    */
   @Override
   public void setPreferredSizeInPixels(int preferredWidth, int preferredHeight)
   {
      this.preferredWidth = preferredWidth;
      this.preferredHeight = preferredHeight;
   }

   /* (non-Javadoc)
    * @see com.mindbook.spriteEngine.SpriteWorldViewer#setSpriteWorld(com.mindbook.spriteEngine.SpriteWorld)
    */
   @Override
   public void setSpriteWorld(SpriteWorld spriteWorld)
   {
      this.spriteWorld = spriteWorld;
      if (spriteWorldPanel != null) spriteWorldPanel.setSpriteWorld(this, spriteWorld);
   }
   
   @Override
   public SpriteWorld getSpriteWorld()
   {
      return spriteWorld;
   }

   /* (non-Javadoc)
    * @see com.mindbook.spriteEngine.SpriteWorldViewer#setResizable(boolean)
    */
   @Override
   public void setResizable(boolean resizable)
   {
      this.resizable = resizable;
   }

   /* (non-Javadoc)
    * @see com.mindbook.spriteEngine.SpriteWorldViewer#createAndDisplayWindow()
    */
   @Override
   public void createAndDisplayWindow()
   {
      jFrame = new JFrame(name);

      if ((startupLocationOnScreenX != -1) && (startupLocationOnScreenY != -1))
      {
         jFrame.setLocation(startupLocationOnScreenX, startupLocationOnScreenY);
      }
      
      spriteWorldPanel = new SpriteWorldPanel(this, spriteWorld);

      jFrame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      Container contentPane = jFrame.getContentPane();

      contentPane.setLayout(new BorderLayout());
      contentPane.add(spriteWorldPanel);

      jFrame.pack();

      jFrame.setSize(preferredWidth, preferredHeight);

      jFrame.setResizable(true); //resizable);
      jFrame.setVisible(true);

      this.update();
   }

   /* (non-Javadoc)
    * @see com.mindbook.spriteEngine.SpriteWorldViewer#addButton(javax.swing.JButton)
    */
   @Override
   public void addButton(JButton button)
   {
      jFrame.getContentPane().add(button, BorderLayout.EAST);

      jFrame.repaint();
//      jFrame.setSize(preferredWidth, preferredHeight);
//
//      jFrame.setResizable(resizable);
//      jFrame.setVisible(true);
//
//      jFrame.pack();
   }

   @Override
   public void addButton(Button button)
   {
   }

   public void update()
   {
      spriteWorldPanel.repaint();
   }

   @Override
   public void setLocationOnScreen(int x, int y)
   {
      if (jFrame != null)
      {
         jFrame.setLocation(x, y);
      }
      else
      {
         startupLocationOnScreenX = x;
         startupLocationOnScreenY = y;
      }
   }

   @Override
   public void exit()
   {
      jFrame.dispose();
   }
}
