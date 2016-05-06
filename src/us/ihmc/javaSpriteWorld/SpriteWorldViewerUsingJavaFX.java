package us.ihmc.javaSpriteWorld;

import java.util.concurrent.CountDownLatch;

import javax.swing.JButton;

import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.transform.Scale;
import javafx.scene.transform.Transform;
import javafx.stage.Stage;

public class SpriteWorldViewerUsingJavaFX implements SpriteWorldViewer
{   
   private final String name;
   private final Group spriteWorldViewerJavaFXGroup = new Group();
   private SpriteWorldJavaFXGroup spriteWorldJavaFXGroup;
   
   private int preferredWidth = 1000, preferredHeight = 1000;
   private boolean resizable = false;
   
   private SpriteWorld spriteWorld;
   
   private final Scale scaleForViewerPixelSize = new Scale();

   public SpriteWorldViewerUsingJavaFX(String name)
   {
      this.name = name;
      JavaFXApplicationCreator.createAJavaFXApplication();
   }
   
   public void setPreferredSizeInPixels(int preferredWidth, int preferredHeight)
   {
      this.preferredWidth = preferredWidth;
      this.preferredHeight = preferredHeight;
   }
   
   public void setSpriteWorld(SpriteWorld spriteWorld)
   {
      this.spriteWorld = spriteWorld;
      
      spriteWorldJavaFXGroup = new SpriteWorldJavaFXGroup(spriteWorld);
      spriteWorldViewerJavaFXGroup.getChildren().add(spriteWorldJavaFXGroup);
   }
   
   public void setResizable(boolean resizable)
   {
      this.resizable = resizable;
   }
       
   public void createAndDisplayWindow()
   {
      final Stage[] stage = new Stage[1];

      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable(){

         @Override
         public void run()
         {
            ObservableList<Transform> transforms = spriteWorldViewerJavaFXGroup.getTransforms();
            transforms.clear();
            
            scaleForViewerPixelSize.setX(preferredWidth);
            scaleForViewerPixelSize.setY(preferredHeight);
            transforms.add(scaleForViewerPixelSize);      
        
//            spriteWorldJavaFXGroup.update();

            stage[0] = new Stage();
            Scene scene = new Scene(spriteWorldViewerJavaFXGroup, preferredWidth, preferredHeight, Color.WHITESMOKE);
            stage[0].setScene(scene);
            stage[0].centerOnScreen();
            stage[0].show();

            countDownLatch.countDown();
         }});
   }
   
//   public void addButton(JButton button)
//   {
//      jFrame.getContentPane().add(button, BorderLayout.EAST);
//      
//      jFrame.repaint();
////      jFrame.setSize(preferredWidth, preferredHeight);
////      
////      jFrame.setResizable(resizable);
////      jFrame.setVisible(true);
////      
////      jFrame.pack();
//   }
   
   
   public void update()
   {
      final CountDownLatch countDownLatch = new CountDownLatch(1);

      boolean fxApplicationThread = Platform.isFxApplicationThread();
//      System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Creating a runnable to run later. fxApplicationThread = " + fxApplicationThread);

      if (fxApplicationThread)
      {
         spriteWorldJavaFXGroup.update();
      }

      else
      {
         Platform.runLater(new Runnable(){

            @Override
            public void run()
            {
//               System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Updating spriteWorldJavaFXGroup.");

               spriteWorldJavaFXGroup.update();

//               System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Done updating spriteWorldJavaFXGroup.");

               countDownLatch.countDown();
            }});

         try
         {

//            System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Awaiting countDownLatch.");

            countDownLatch.await();

//            System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Done awaiting countDownLatch.");

         }
         catch (InterruptedException e)
         {
//            System.err.println("Exception " + e);
         }
      }
   }

   public void addButton(JButton realTimeButton)
   {      
   }
}
