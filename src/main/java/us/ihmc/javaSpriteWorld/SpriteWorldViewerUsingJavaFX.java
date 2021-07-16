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
import us.ihmc.javaSpriteWorld.javaFX.JavaFXApplicationCreator;
import us.ihmc.javaSpriteWorld.javaFX.SpriteWorldJavaFXGroup;

public class SpriteWorldViewerUsingJavaFX implements SpriteWorldViewer
{
   private final String name;
   private final Group spriteWorldViewerJavaFXGroup = new Group();
   private SpriteWorldJavaFXGroup spriteWorldJavaFXGroup;

   private int preferredWidth = 1000, preferredHeight = 1000;
   private boolean resizable = false;

   private SpriteWorld spriteWorld;

   private final Scale scaleForViewerPixelSize = new Scale();
   
   private Scene scene;
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
   
   public Scene getScene()
   {
      return scene;
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
            spriteWorldJavaFXGroup.requestFocus();
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

   public void createScene()
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
            scene = new Scene(spriteWorldViewerJavaFXGroup, preferredWidth, preferredHeight, Color.WHITESMOKE);

            countDownLatch.countDown();
         }
      });

      try
      {
         countDownLatch.await();
      }
      catch (InterruptedException e)
      {
      }
      
   }

   @Override
   public void createAndDisplayWindow()
   {
      if (scene == null)
      {
         createScene();
      }
      
      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            stage = new Stage();
            stage.setTitle(name);
            stage.setScene(scene);

            if ((startupLocationOnScreenX != -1) && (startupLocationOnScreenY != -1))
            {
               stage.setX(startupLocationOnScreenX);
               stage.setY(startupLocationOnScreenY);
            }
            else
            {
               stage.centerOnScreen();
            }

            stage.show();

            if (spriteWorldJavaFXGroup != null)
               spriteWorldJavaFXGroup.requestFocus();

            countDownLatch.countDown();
         }
      });

      try
      {
         countDownLatch.await();
      }
      catch (InterruptedException e)
      {
      }

      update();

   }

   //   private void printAndPause(String string, long pause)
   //   {
   //      System.out.println(string);
   //      try
   //      {
   //         Thread.sleep(pause);
   //      }
   //      catch (InterruptedException e)
   //      {
   //      }
   //   }

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

   @Override
   public void exit()
   {
      stage.close();
   }

}
