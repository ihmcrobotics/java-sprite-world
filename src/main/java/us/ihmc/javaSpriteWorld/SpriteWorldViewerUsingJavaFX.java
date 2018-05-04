package us.ihmc.javaSpriteWorld;

import java.util.concurrent.CountDownLatch;

import javax.swing.JButton;

import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.control.Button;
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
   private Stage stage;

   private int startupLocationOnScreenX = -1;
   private int startupLocationOnScreenY = -1;

   public SpriteWorldViewerUsingJavaFX(String name)
   {
      this.name = name;
      JavaFXApplicationCreator.createAJavaFXApplication();
   }

   public Stage getStage()
   {
      return stage;
   }

   public void setPreferredSizeInPixels(int preferredWidth, int preferredHeight)
   {
      this.preferredWidth = preferredWidth;
      this.preferredHeight = preferredHeight;
   }

   @Override
   public void setSpriteWorld(SpriteWorld spriteWorld)
   {
      this.spriteWorld = spriteWorld;

      spriteWorldJavaFXGroup = new SpriteWorldJavaFXGroup(this, spriteWorld);

      Platform.runLater(new Runnable()
      {

         @Override
         public void run()
         {
            spriteWorldViewerJavaFXGroup.getChildren().add(spriteWorldJavaFXGroup);
         }
      });
   }

   @Override
   public SpriteWorld getSpriteWorld()
   {
      return spriteWorld;
   }

   public void setResizable(boolean resizable)
   {
      this.resizable = resizable;
   }

   @Override
   public void createAndDisplayWindow()
   {
      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            ObservableList<Transform> transforms = spriteWorldViewerJavaFXGroup.getTransforms();
            transforms.clear();

            scaleForViewerPixelSize.setX(preferredWidth);
            scaleForViewerPixelSize.setY(preferredHeight);
            transforms.add(scaleForViewerPixelSize);

            //            spriteWorldJavaFXGroup.update();

            //            printAndPause("Creating Stage", 2000L);
            stage = new Stage();

            if ((startupLocationOnScreenX != -1) && (startupLocationOnScreenY != -1))
            {
               stage.setX(startupLocationOnScreenX);
               stage.setY(startupLocationOnScreenY);
            }
            else
            {
               stage.centerOnScreen();
            }

            Scene scene = new Scene(spriteWorldViewerJavaFXGroup, preferredWidth, preferredHeight, Color.WHITESMOKE);
            stage.setScene(scene);

            //            printAndPause("Showing Stage", 2000L);
            stage.show();

            //            printAndPause("Done Showing Stage", 2000L);

            countDownLatch.countDown();
         }
      });

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

      //      printAndPause("Updating Stage", 2000L);

      update();

      //      printAndPause("Done Updating Stage", 2000L);

   }

   private void printAndPause(String string, long pause)
   {
      System.out.println(string);
      try
      {
         Thread.sleep(pause);
      }
      catch (InterruptedException e)
      {
      }
   }

   public void update()
   {
      final CountDownLatch countDownLatch = new CountDownLatch(1);

      boolean fxApplicationThread = Platform.isFxApplicationThread();

      if (fxApplicationThread)
      {
         spriteWorldJavaFXGroup.update();
      }

      else
      {
         Platform.runLater(new Runnable()
         {

            @Override
            public void run()
            {
               //               System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Updating spriteWorldJavaFXGroup.");

               spriteWorldJavaFXGroup.update();

               //               System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Done updating spriteWorldJavaFXGroup.");

               countDownLatch.countDown();
            }
         });

         try
         {

            //            System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Awaiting countDownLatch.");

            countDownLatch.await();

            //            System.out.println("In SpriteWorldViewerUsingJavaFX.update(). Done awaiting countDownLatch.");

         }
         catch (InterruptedException e)
         {
         }
      }
   }

   @Override
   public void addButton(JButton realTimeButton)
   {
   }

   @Override
   public void addButton(Button button)
   {
   }

   @Override
   public void setLocationOnScreen(int x, int y)
   {
      if (stage != null)
      {
         stage.setX(x);
         stage.setY(y);
      }
      else
      {
         startupLocationOnScreenX = x;
         startupLocationOnScreenY = y;
      }
   }

}
