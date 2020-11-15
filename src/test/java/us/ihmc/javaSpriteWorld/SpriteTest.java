package us.ihmc.javaSpriteWorld;

import java.util.ArrayList;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.javaSpriteWorld.geometry.AxisAlignedBoundingBox2D;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygonIntersectionDetector;
import us.ihmc.javaSpriteWorld.geometry.Point;

import static org.junit.jupiter.api.Assertions.*;

@Tag("gui")
public class SpriteTest
{

   @Test
   public void testSprite()
   {
      Sprite spriteOne = new Sprite("SpriteOne");
      Sprite spriteTwo = new Sprite("SpriteTwo");
      
      ArrayList<Point> points = new ArrayList<>();
      
      points.add(new Point(-1.0, -1.0));
      points.add(new Point(1.0, -1.0));
      points.add(new Point(1.0, 1.0));
      points.add(new Point(-1.0, 1.0));
      
      ConvexPolygon collisionPolygonOne = new ConvexPolygon(points);
      spriteOne.addCollisionPolygon(collisionPolygonOne);
      
      ConvexPolygon collisionPolygonTwo = new ConvexPolygon(points);
      spriteTwo.addCollisionPolygon(collisionPolygonTwo);
      
      ArrayList<ConvexPolygon> collisionPolygonsOne = new ArrayList<>();
      spriteOne.getCollisionPolygons(collisionPolygonsOne);
      
      ArrayList<ConvexPolygon> collisionPolygonsTwo = new ArrayList<>();
      spriteTwo.getCollisionPolygons(collisionPolygonsTwo);
      
      ConvexPolygonIntersectionDetector intersectionDetector = new ConvexPolygonIntersectionDetector();
      boolean arePolygonsIntersecting = intersectionDetector.arePolygonsIntersecting(collisionPolygonsOne.get(0), collisionPolygonsOne.get(0));
      assertTrue(arePolygonsIntersecting);

      spriteOne.setRotationInDegrees(90.0);

      double actual11 = spriteOne.getRotationInDegrees();
      assertEquals(90.0, actual11, 1e-7);
      double actual10 = spriteOne.getRotationInRadians();
      assertEquals(Math.PI / 2.0, actual10, 1e-7);

      spriteOne.setRotationInRadians(Math.PI);
      double actual9 = spriteOne.getRotationInDegrees();
      assertEquals(180.0, actual9, 1e-7);
      double actual8 = spriteOne.getRotationInRadians();
      assertEquals(Math.PI, actual8, 1e-7);

      spriteOne.setX(20.0);
      
      collisionPolygonsOne.clear();
      collisionPolygonsTwo.clear();
      
      spriteOne.getCollisionPolygons(collisionPolygonsOne);
      spriteTwo.getCollisionPolygons(collisionPolygonsTwo);

      arePolygonsIntersecting = intersectionDetector.arePolygonsIntersecting(collisionPolygonsOne.get(0), collisionPolygonsTwo.get(0));
      assertFalse(arePolygonsIntersecting);

      AxisAlignedBoundingBox2D boundingBoxOne = new AxisAlignedBoundingBox2D();
      spriteOne.getAxisAlignedBoundingBox(boundingBoxOne);
      
      AxisAlignedBoundingBox2D boundingBoxTwo = new AxisAlignedBoundingBox2D();
      spriteTwo.getAxisAlignedBoundingBox(boundingBoxTwo);

      double actual7 = boundingBoxOne.getMinX();
      assertEquals(19.0, actual7, 1e-7);
      double actual6 = boundingBoxOne.getMinY();
      assertEquals(-1.0, actual6, 1e-7);
      double actual5 = boundingBoxOne.getMaxX();
      assertEquals(21.0, actual5, 1e-7);
      double actual4 = boundingBoxOne.getMaxY();
      assertEquals(1.0, actual4, 1e-7);

      double actual3 = boundingBoxTwo.getMinX();
      assertEquals(-1.0, actual3, 1e-7);
      double actual2 = boundingBoxTwo.getMinY();
      assertEquals(-1.0, actual2, 1e-7);
      double actual1 = boundingBoxTwo.getMaxX();
      assertEquals(1.0, actual1, 1e-7);
      double actual = boundingBoxTwo.getMaxY();
      assertEquals(1.0, actual, 1e-7);
   }
}
