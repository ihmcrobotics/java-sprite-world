package us.ihmc.javaSpriteWorld.geometry;

import java.util.ArrayList;

/**
 * Detects if Polygons are intersecting. Uses the separating axis theorem.
 * @author Jerry Pratt
 *
 */
public class ConvexPolygonIntersectionDetector
{
   private final ArrayList<Vector> edgeNormals = new ArrayList<>();
   private final double[] extentsOne = new double[2];
   private final double[] extentsTwo = new double[2];

   public boolean arePolygonsIntersecting(ConvexPolygon polygonOne, ConvexPolygon polygonTwo)
   {
      edgeNormals.clear();
      polygonOne.getEdgeNormals(edgeNormals);
      polygonTwo.getEdgeNormals(edgeNormals);

      ArrayList<Point> verticesOne = polygonOne.getVertices();
      ArrayList<Point> verticesTwo = polygonTwo.getVertices();

      for (int i = 0; i < edgeNormals.size(); i++)
      {
         Vector edgeNormal = edgeNormals.get(i);

         if (isASeparatingAxis(verticesOne, verticesTwo, edgeNormal)) return false;
      }

      return true;
   }
   
   public boolean isFirstPolygonFullyInsideSecondPolygon(ConvexPolygon polygonOne, ConvexPolygon polygonTwo)
   {
      edgeNormals.clear();
      polygonOne.getEdgeNormals(edgeNormals);
      polygonTwo.getEdgeNormals(edgeNormals);

      ArrayList<Point> verticesOne = polygonOne.getVertices();
      ArrayList<Point> verticesTwo = polygonTwo.getVertices();

      for (int i = 0; i < edgeNormals.size(); i++)
      {
         Vector edgeNormal = edgeNormals.get(i);

         if (!isAFullyContainedAxis(verticesOne, verticesTwo, edgeNormal)) return false;
      }

      return true;
   }

   private boolean isASeparatingAxis(ArrayList<Point> verticesOne, ArrayList<Point> verticesTwo, Vector edgeNormal)
   {
      computeVertexProjectionsOntoEdgeNormalLine(verticesOne, edgeNormal, extentsOne);
      computeVertexProjectionsOntoEdgeNormalLine(verticesTwo, edgeNormal, extentsTwo);

      double minOne = extentsOne[0];
      double maxOne = extentsOne[1];

      double minTwo = extentsTwo[0];
      double maxTwo = extentsTwo[1];

      if (maxOne < minTwo)
         return true;
      if (maxTwo < minOne)
         return true;
      
      return false;
   }
   
   private boolean isAFullyContainedAxis(ArrayList<Point> verticesOne, ArrayList<Point> verticesTwo, Vector edgeNormal)
   {
      computeVertexProjectionsOntoEdgeNormalLine(verticesOne, edgeNormal, extentsOne);
      computeVertexProjectionsOntoEdgeNormalLine(verticesTwo, edgeNormal, extentsTwo);

      double minOne = extentsOne[0];
      double maxOne = extentsOne[1];

      double minTwo = extentsTwo[0];
      double maxTwo = extentsTwo[1];

      return ((maxOne < maxTwo) && (minOne > minTwo));
   }

   private void computeVertexProjectionsOntoEdgeNormalLine(ArrayList<Point> vertices, Vector edgeNormal, double[] extents)
   {
      double minDotProduct = Double.POSITIVE_INFINITY;
      double maxDotProduct = Double.NEGATIVE_INFINITY;

      for (int i = 0; i < vertices.size(); i++)
      {
         Point vertex = vertices.get(i);

         double dot = vertex.getX() * edgeNormal.getX() + vertex.getY() * edgeNormal.getY();

         if (dot > maxDotProduct)
            maxDotProduct = dot;
         if (dot < minDotProduct)
            minDotProduct = dot;
      }

      extents[0] = minDotProduct;
      extents[1] = maxDotProduct;
   }

}
