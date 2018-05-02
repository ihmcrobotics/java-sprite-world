package us.ihmc.javaSpriteWorld.geometry;

import static org.junit.Assert.*;

import org.junit.Test;

import us.ihmc.javaSpriteWorld.geometry.Point;

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
      
      assertEquals(point.getX(), 3.5, 1e-7);
      assertEquals(point.getY(), 6.1, 1e-7);
      
      
      point.set(1.0, 0.0);
      
      xTranslation = 0.0;
      yTranslation = 0.0;
      rotation = Math.PI/2.0;
      point.transform(false, false, xTranslation, yTranslation, rotation);
      
      assertEquals(point.getX(), 0.0, 1e-7);
      assertEquals(point.getY(), 1.0, 1e-7);
   }

}
