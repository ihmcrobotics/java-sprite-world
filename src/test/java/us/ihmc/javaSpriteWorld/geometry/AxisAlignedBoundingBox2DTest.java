package us.ihmc.javaSpriteWorld.geometry;

import static us.ihmc.robotics.Assert.*;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.javaSpriteWorld.geometry.AxisAlignedBoundingBox2D;
import us.ihmc.javaSpriteWorld.geometry.Point;

@Tag("gui")
public class AxisAlignedBoundingBox2DTest
{

   @Test
   public void testBoundingBox()
   {
      AxisAlignedBoundingBox2D boundingBox = new AxisAlignedBoundingBox2D();
      
      assertTrue(boundingBox.isNaN());
      
      boundingBox.expandToInclude(new Point(1.0, 2.0));
      
      assertFalse(boundingBox.isNaN());
      
      boundingBox.expandToInclude(new Point(1.5, 0.1));
      
      Point minPoint = new Point();
      boundingBox.getMinPoint(minPoint);
      
      Point maxPoint = new Point();
      boundingBox.getMaxPoint(maxPoint);
      
      assertEquals(1.0, minPoint.getX(), 1e-7);
      assertEquals(0.1, minPoint.getY(), 1e-7);
      
      assertEquals(1.5, maxPoint.getX(), 1e-7);
      assertEquals(2.0, maxPoint.getY(), 1e-7);
   }
   
   @Test
   public void testIsEntirelyInside()
   {
      AxisAlignedBoundingBox2D boundingBoxOne = new AxisAlignedBoundingBox2D();
      AxisAlignedBoundingBox2D boundingBoxTwo = new AxisAlignedBoundingBox2D();
      
      assertFalse(boundingBoxOne.isEntirelyInside(boundingBoxTwo));
      assertFalse(boundingBoxTwo.isEntirelyInside(boundingBoxOne));
      
      boundingBoxOne.expandToInclude(new Point(0.0, 0.0));
      boundingBoxTwo.expandToInclude(new Point(0.5, 0.5));
      
      assertFalse(boundingBoxOne.isEntirelyInside(boundingBoxTwo));
      assertFalse(boundingBoxTwo.isEntirelyInside(boundingBoxOne));

      boundingBoxOne.expandToInclude(new Point(1.0, 1.0));
      assertFalse(boundingBoxOne.isEntirelyInside(boundingBoxTwo));
      assertTrue(boundingBoxTwo.isEntirelyInside(boundingBoxOne));

      boundingBoxTwo.expandToInclude(new Point(0.75, 0.75));
      assertFalse(boundingBoxOne.isEntirelyInside(boundingBoxTwo));
      assertTrue(boundingBoxTwo.isEntirelyInside(boundingBoxOne));
      
      boundingBoxTwo.expandToInclude(new Point(1.75, 1.75));
      assertFalse(boundingBoxOne.isEntirelyInside(boundingBoxTwo));
      assertFalse(boundingBoxTwo.isEntirelyInside(boundingBoxOne));
      
      boundingBoxTwo.expandToInclude(new Point(-1.0,-1.0));
      assertTrue(boundingBoxOne.isEntirelyInside(boundingBoxTwo));
      assertFalse(boundingBoxTwo.isEntirelyInside(boundingBoxOne));
      
      
   }

}
