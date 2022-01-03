package us.ihmc.javaSpriteWorld;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

@Tag("gui")
public class SpriteWorldViewerUsingLibGDXTest
{
   @Test
   public void testSimpleSpriteWorldViewer() throws InterruptedException
   {
      SpriteWorldViewerUsingLibGDX viewer = new SpriteWorldViewerUsingLibGDX("Test Using libGDX");

      viewer.setPreferredSizeInPixels(300, 300);
      viewer.setResizable(false);

      SpriteWorld spriteWorld = new SpriteWorld();
      spriteWorld.setLeftBorderX(0.0);
      spriteWorld.setTopBorderY(0.0);
      spriteWorld.setRightBorderX(300.0);
      spriteWorld.setBottomBorderY(300.0);

      Sprite crossHairs = new Sprite("CrossHairs");

      crossHairs.addCostume(SampleSpriteCostumes.getCrossHairs());
      crossHairs.setWidth(300.0);
      crossHairs.setHeight(300.0);

      crossHairs.setX(150.0);
      crossHairs.setY(150.0);

      crossHairs.setRotationInDegrees(5.0);

      spriteWorld.addSprite(crossHairs);

      viewer.setSpriteWorld(spriteWorld);
      viewer.createAndDisplayWindow();

      double rotation = 5.0;

      int numberOfLoops = 20;

      for (int i = 0; i < numberOfLoops; i++)
      {
         rotation = rotation + 5.0;
         crossHairs.setRotationInDegrees(rotation);
         viewer.update();
         Thread.sleep(100L);
      }

      Thread.sleep(10000000L);
   }
}
