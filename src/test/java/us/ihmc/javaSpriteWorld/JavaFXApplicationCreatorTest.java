package us.ihmc.javaSpriteWorld;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.scene.Group;
import javafx.scene.Scene;
import javafx.scene.paint.Color;
import javafx.scene.shape.Circle;
import javafx.scene.transform.Translate;
import javafx.stage.Stage;
import us.ihmc.javaSpriteWorld.JavaFXApplicationCreator;

@Tag("gui")
public class JavaFXApplicationCreatorTest
{
   @Test
   public void testJavaFXApplicationCreator() throws InterruptedException
   {
      for (int i=0; i<5; i++)
      {
         createACircleAnimation(i);
      }

      Thread.sleep(5000L);
   }

   private void createACircleAnimation(final int index)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      final Translate circleTranslation = new Translate();

      Platform.runLater(new Runnable()
      {
         @Override
         public void run()
         {
            Stage stage = new Stage();

            double preferredWidth = 500;
            double preferredHeight = 500;

            Circle circle = new Circle();

            circle.setRadius(50.0f);
            circle.setCenterX(25.0);
            circle.setCenterY(25.0);

            Group circleGroup = new Group();
            circleGroup.getTransforms().add(circleTranslation);
            circleGroup.getChildren().add(circle);

            Scene scene = new Scene(circleGroup, preferredWidth, preferredHeight, Color.WHITESMOKE);
            stage.setScene(scene);
            
//            stage.centerOnScreen();
            stage.setX(250.0 * index);
            stage.setY(250.0 * index);
            
            stage.setTitle("Cirlce Animation");
            
            stage.show();
         }
      });

      final long startTime = System.nanoTime();

      AnimationTimer animationTimer = new AnimationTimer()
      {
         double freq = 2.0 * Math.random();
         
         @Override
         public void handle(long arg0)
         {
            double t = (double) ((System.nanoTime() - startTime) * 1e-9);

            double x = 150.0 + 100.0 * Math.sin(2.0 * Math.PI * freq * t);
            double y = 150.0 + 100.0 * Math.cos(2.0 * Math.PI * freq * t);

            circleTranslation.setX(x);
            circleTranslation.setY(y);
         }
      };

      animationTimer.start();
   }
}
