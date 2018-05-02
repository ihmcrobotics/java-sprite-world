package us.ihmc.javaSpriteWorld;

import java.util.concurrent.CountDownLatch;

import org.junit.Test;

import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.image.Image;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.javaSpriteWorld.JavaFXApplicationCreator;
import us.ihmc.javaSpriteWorld.SpriteCostume;

public class SpriteCostumeTest
{
   @Test
   public void testLoadingASpriteCostume() throws InterruptedException
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      String filename = "sampleImages/dice/SixSider/blackOne-300px.png";
//      String filename = "sampleImages/BackgammonBoard.jpg";
      
      SpriteCostume costume = SpriteCostume.createFromFile(filename);
      final Node node = costume.getJavaFXNode();
      
//      System.out.println(costume.getImageHeightPixels());

      final Stage[] stage = new Stage[1];
      
      final CountDownLatch countDownLatch = new CountDownLatch(1);
      
      Platform.runLater(new Runnable(){

         @Override
         public void run()
         {
            stage[0] = new Stage();
            Group rootGroup = new Group();
            Scene scene = new Scene(rootGroup, 200, 200, Color.WHITESMOKE);
            stage[0].setScene(scene);
            stage[0].centerOnScreen();
            stage[0].show();
            
            rootGroup.getChildren().add(node);
            
            countDownLatch.countDown();
         }});

      
      countDownLatch.await();
      
      Thread.sleep(2000L);
      
      Platform.runLater(new Runnable(){

         @Override
         public void run()
         {
            stage[0].close();
         }});
   }
   
}
