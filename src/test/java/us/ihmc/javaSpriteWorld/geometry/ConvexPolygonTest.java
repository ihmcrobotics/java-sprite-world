package us.ihmc.javaSpriteWorld.geometry;

import java.util.ArrayList;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

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

      double actual7 = edgeNormals.get(0).getX();
      assertEquals(0.0, actual7, 1e-7);
      double actual6 = edgeNormals.get(0).getY();
      assertEquals(-1.0, actual6, 1e-7);

      double actual5 = edgeNormals.get(1).getX();
      assertEquals(1.0, actual5, 1e-7);
      double actual4 = edgeNormals.get(1).getY();
      assertEquals(0.0, actual4, 1e-7);

      double actual3 = edgeNormals.get(2).getX();
      assertEquals(0.0, actual3, 1e-7);
      double actual2 = edgeNormals.get(2).getY();
      assertEquals(1.0, actual2, 1e-7);

      double actual1 = edgeNormals.get(3).getX();
      assertEquals(-1.0, actual1, 1e-7);
      double actual = edgeNormals.get(3).getY();
      assertEquals(0.0, actual, 1e-7);
   }

}
