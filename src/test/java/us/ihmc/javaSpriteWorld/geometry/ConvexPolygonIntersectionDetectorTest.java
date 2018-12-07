package us.ihmc.javaSpriteWorld.geometry;

import static us.ihmc.robotics.Assert.*;

import java.util.ArrayList;

import org.junit.jupiter.api.Test;

import us.ihmc.javaSpriteWorld.geometry.ConvexPolygon;
import us.ihmc.javaSpriteWorld.geometry.ConvexPolygonIntersectionDetector;
import us.ihmc.javaSpriteWorld.geometry.Point;

public class ConvexPolygonIntersectionDetectorTest
{

   @Test
   public void testPolygonIntersectionDetector()
   {
      ConvexPolygonIntersectionDetector detector = new ConvexPolygonIntersectionDetector();

      ArrayList<Point> pointsOne = new ArrayList<>();

      pointsOne.add(new Point(0.0, 0.0));
      pointsOne.add(new Point(1.0, 0.0));
      pointsOne.add(new Point(1.0, 1.0));
      pointsOne.add(new Point(0.0, 1.0));

      ConvexPolygon polygonOne = new ConvexPolygon(pointsOne);
      
      ArrayList<Point> pointsTwo = new ArrayList<>();

      pointsTwo.add(new Point(0.0, 0.0));
      pointsTwo.add(new Point(1.0, 0.0));
      pointsTwo.add(new Point(1.0, 1.0));
      pointsTwo.add(new Point(0.0, 1.0));

      ConvexPolygon polygonTwo = new ConvexPolygon(pointsTwo);
      
      boolean intersecting = detector.arePolygonsIntersecting(polygonOne, polygonTwo);
      boolean firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonOne, polygonTwo);
      assertTrue(intersecting);
      assertFalse(firstIsFullyInsideSecond);

      ArrayList<Point> pointsThree = new ArrayList<>();

      pointsThree.add(new Point(1.1, 0.0));
      pointsThree.add(new Point(2.1, 0.0));
      pointsThree.add(new Point(2.1, 1.0));
      pointsThree.add(new Point(1.1, 1.0));

      ConvexPolygon polygonThree = new ConvexPolygon(pointsThree);
      
      intersecting = detector.arePolygonsIntersecting(polygonOne, polygonThree);
      firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonOne, polygonThree);
      assertFalse(intersecting);
      assertFalse(firstIsFullyInsideSecond);
      
      ArrayList<Point> pointsFour = new ArrayList<>();

      pointsFour.add(new Point(21.0, 1.0));
      pointsFour.add(new Point(19.0, 1.0));
      pointsFour.add(new Point(19.0, -0.99));
      pointsFour.add(new Point(21.0, -1.0));

      ConvexPolygon polygonFour = new ConvexPolygon(pointsFour);

      intersecting = detector.arePolygonsIntersecting(polygonOne, polygonFour);
      firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonOne, polygonFour);
      assertFalse(intersecting);
      assertFalse(firstIsFullyInsideSecond);

      intersecting = detector.arePolygonsIntersecting(polygonTwo, polygonFour);
      firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonTwo, polygonFour);
      assertFalse(intersecting);
      assertFalse(firstIsFullyInsideSecond);

      intersecting = detector.arePolygonsIntersecting(polygonThree, polygonFour);
      firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonThree, polygonFour);
      assertFalse(intersecting);
      assertFalse(firstIsFullyInsideSecond);

      intersecting = detector.arePolygonsIntersecting(polygonFour, polygonFour);
      firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonFour, polygonFour);
      assertTrue(intersecting);
      assertFalse(firstIsFullyInsideSecond);
      
      ArrayList<Point> pointsFive = new ArrayList<>();

      pointsOne.add(new Point(0.5, 0.5));
      pointsOne.add(new Point(0.75, 0.5));
      pointsOne.add(new Point(0.75, 0.75));
      pointsOne.add(new Point(0.5, 0.75));

      ConvexPolygon polygonFive = new ConvexPolygon(pointsFive);
      
      firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonFive, polygonOne);
      assertTrue(firstIsFullyInsideSecond);
      firstIsFullyInsideSecond = detector.isFirstPolygonFullyInsideSecondPolygon(polygonOne, polygonFive);
      assertFalse(firstIsFullyInsideSecond);

   }

}
