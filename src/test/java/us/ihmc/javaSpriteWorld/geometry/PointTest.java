package us.ihmc.javaSpriteWorld.geometry;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

@Tag("gui")
public class PointTest
{

   @Test
   public void testPoint()
   {
      Point point = new Point(3.0, 4.0);
      
      double xTranslation = 0.5;
      double yTranslation = 2.1;
      double rotation = 0.0;
      
      point.transform(false, false, xTranslation, yTranslation, rotation);

      double expected3 = point.getX();
      assertEquals(expected3, 3.5, 1e-7);
      double expected2 = point.getY();
      assertEquals(expected2, 6.1, 1e-7);

      point.set(1.0, 0.0);
      
      xTranslation = 0.0;
      yTranslation = 0.0;
      rotation = Math.PI/2.0;
      point.transform(false, false, xTranslation, yTranslation, rotation);

      double expected1 = point.getX();
      assertEquals(expected1, 0.0, 1e-7);
      double expected = point.getY();
      assertEquals(expected, 1.0, 1e-7);
   }

}
