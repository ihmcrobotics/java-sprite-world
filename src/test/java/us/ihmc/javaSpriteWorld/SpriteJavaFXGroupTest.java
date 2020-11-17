package us.ihmc.javaSpriteWorld;

import java.util.concurrent.CountDownLatch;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.stage.Stage;

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

      Platform.runLater(() ->
      {
         stage[0] = new Stage();
         Scene scene = new Scene(group, 200, 200, Color.WHITESMOKE);
         stage[0].setScene(scene);
         stage[0].centerOnScreen();
         stage[0].show();

         countDownLatch.countDown();
      });

      countDownLatch.await();

      for (int i=0; i<10; i++)
      {
         sprite.nextCostume();

         Platform.runLater(group::update);

         Thread.sleep(100L);
      }

      Platform.runLater(() -> stage[0].close());
   }
}
