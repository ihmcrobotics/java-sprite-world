package us.ihmc.javaSpriteWorld;

import static us.ihmc.robotics.Assert.*;

import java.util.concurrent.CountDownLatch;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.javaSpriteWorld.JavaFXApplicationCreator;
import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteJavaFXGroup;

@Tag("gui")
public class SpriteJavaFXGroupTest
{

   @Test
   public void testSpriteJavaFXGroup() throws InterruptedException
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      Sprite sprite = SampleSprites.createSixSidedBlackPipsOnWhiteDie();
      final SpriteJavaFXGroup group = new SpriteJavaFXGroup(sprite);

      final Stage[] stage = new Stage[1];

      final CountDownLatch countDownLatch = new CountDownLatch(1);

      Platform.runLater(new Runnable(){

         @Override
         public void run()
         {
            stage[0] = new Stage();
            Scene scene = new Scene(group, 200, 200, Color.WHITESMOKE);
            stage[0].setScene(scene);
            stage[0].centerOnScreen();
            stage[0].show();

            countDownLatch.countDown();
         }});

      countDownLatch.await();

      for (int i=0; i<10; i++)
      {
         sprite.nextCostume();

         Platform.runLater(new Runnable(){
            @Override
            public void run()
            {
               group.update();
            }});

         Thread.sleep(100L);
      }

      Platform.runLater(new Runnable(){

         @Override
         public void run()
         {
            stage[0].close();
         }});
   }

}
