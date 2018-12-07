package us.ihmc.javaSpriteWorld;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.javaSpriteWorld.Sprite;
import us.ihmc.javaSpriteWorld.geometry.AxisAlignedBoundingBox2D;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygonIntersectionDetector;
import us.ihmc.javaSpriteWorld.geometry.Point;

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
      
      
      assertEquals(90.0, spriteOne.getRotationInDegrees(), 1e-7);
      assertEquals(Math.PI/2.0, spriteOne.getRotationInRadians(), 1e-7);
      
      spriteOne.setRotationInRadians(Math.PI);
      assertEquals(180.0, spriteOne.getRotationInDegrees(), 1e-7);
      assertEquals(Math.PI, spriteOne.getRotationInRadians(), 1e-7);
      
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
      
      assertEquals(19.0, boundingBoxOne.getMinX(), 1e-7);
      assertEquals(-1.0, boundingBoxOne.getMinY(), 1e-7);
      assertEquals(21.0, boundingBoxOne.getMaxX(), 1e-7);
      assertEquals(1.0, boundingBoxOne.getMaxY(), 1e-7);
      
      assertEquals(-1.0, boundingBoxTwo.getMinX(), 1e-7);
      assertEquals(-1.0, boundingBoxTwo.getMinY(), 1e-7);
      assertEquals(1.0, boundingBoxTwo.getMaxX(), 1e-7);
      assertEquals(1.0, boundingBoxTwo.getMaxY(), 1e-7);

   }
}
