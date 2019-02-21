package us.ihmc.javaSpriteWorld.geometry;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.Point;
import us.ihmc.javaSpriteWorld.geometry.Vector;

@Tag("gui")
public class ConvexPolygonTest
{

   @Test
   public void testConvexPolygon()
   {
      ArrayList<Point> points = new ArrayList<>();
      
      points.add(new Point(0.0, 0.0));
      points.add(new Point(2.0, 0.0));
      points.add(new Point(2.0, 2.0));
      points.add(new Point(0.0, 2.0));
      
      ConvexPolygon convexPolygon = new ConvexPolygon(points);
      
      ArrayList<Vector> edgeNormals = new ArrayList<>();
      convexPolygon.getEdgeNormals(edgeNormals);
      
      assertEquals(0.0, edgeNormals.get(0).getX(), 1e-7);
      assertEquals(-1.0, edgeNormals.get(0).getY(), 1e-7);
      
      assertEquals(1.0, edgeNormals.get(1).getX(), 1e-7);
      assertEquals(0.0, edgeNormals.get(1).getY(), 1e-7);
      
      assertEquals(0.0, edgeNormals.get(2).getX(), 1e-7);
      assertEquals(1.0, edgeNormals.get(2).getY(), 1e-7);
      
      assertEquals(-1.0, edgeNormals.get(3).getX(), 1e-7);
      assertEquals(0.0, edgeNormals.get(3).getY(), 1e-7);
 
   }

}
