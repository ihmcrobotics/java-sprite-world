package us.ihmc.javaSpriteWorld;

import java.util.concurrent.CountDownLatch;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import javafx.application.Platform;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.javaSpriteWorld.JavaFXApplicationCreator;
import us.ihmc.javaSpriteWorld.SampleSprites;
import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.SpriteWorld;
import us.ihmc.javaSpriteWorld.SpriteWorldJavaFXGroup;

@Tag("gui")
public class SpriteWorldJavaFXGroupTest
{

   @Test
   public void testSpriteWorldJavaFXGroup() throws InterruptedException
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      SpriteWorld spriteWorld = new SpriteWorld();
      
      Sprite spriteOne = SampleSprites.createSixSidedRedPipsOnWhiteDie();
      Sprite spriteTwo = SampleSprites.createSixSidedBlackPipsOnWhiteDie();
      
      spriteWorld.addSprite(spriteOne);
      spriteWorld.addSprite(spriteTwo);
      
      final SpriteWorldJavaFXGroup spriteWorldJavaFXGroup = new SpriteWorldJavaFXGroup(null, spriteWorld);
      
      
      final Stage[] stage = new Stage[1];
      
      final CountDownLatch countDownLatch = new CountDownLatch(1);
      
      Platform.runLater(new Runnable(){

         @Override
         public void run()
         {
            spriteWorldJavaFXGroup.update();
            
            stage[0] = new Stage();
            Scene scene = new Scene(spriteWorldJavaFXGroup, 1000, 1000, Color.WHITESMOKE);
            stage[0].setScene(scene);
            stage[0].centerOnScreen();
            stage[0].show();
            
            countDownLatch.countDown();
         }});

      
      countDownLatch.await();
      
      for (int i=0; i<10; i++)
      {
         spriteOne.nextCostume();
         spriteTwo.previousCostume();
         
         spriteOne.addRotationInRadians(0.01);
         spriteTwo.addRotationInRadians(-0.01);

         Platform.runLater(new Runnable(){
            @Override
            public void run()
            {
               spriteWorldJavaFXGroup.update();
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
