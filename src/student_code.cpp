#include "student_code.h"
#include "mutablePriorityQueue.h"

using namespace std;

namespace CGL
{

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (class member).
   *
   * @param points A vector of points in 2D
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector2D> BezierCurve::evaluateStep(std::vector<Vector2D> const &points)
  { 
    // TODO Part 1.
    vector<Vector2D> evaluatedPoints;
    for (int i = 0; i < points.size() - 1; i++) {
      evaluatedPoints.push_back((1 - t) * points[i] + t * points[i + 1]);
    }
    return evaluatedPoints;
  }

  /**
   * Evaluates one step of the de Casteljau's algorithm using the given points and
   * the scalar parameter t (function parameter).
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return A vector containing intermediate points or the final interpolated vector
   */
  std::vector<Vector3D> BezierPatch::evaluateStep(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    vector<Vector3D> evaluatedPoints;
    for (int i = 0; i < points.size() - 1; i++) {
      evaluatedPoints.push_back((1 - t) * points[i] + t * points[i + 1]);
    }
    return evaluatedPoints;
  }

  /**
   * Fully evaluates de Casteljau's algorithm for a vector of points at scalar parameter t
   *
   * @param points    A vector of points in 3D
   * @param t         Scalar interpolation parameter
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate1D(std::vector<Vector3D> const &points, double t) const
  {
    // TODO Part 2.
    vector<Vector3D> tempPoints = points;
    while (tempPoints.size() > 1) {
      tempPoints = evaluateStep(tempPoints, t);
    }
    return tempPoints[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
    vector<Vector3D> intermediatePoints;
    for (int i = 0; i < this->controlPoints.size(); i++)
    {
      intermediatePoints.push_back(evaluate1D(this->controlPoints[i], u));
    }
    return evaluate1D(intermediatePoints, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.
    HalfedgeCIter h = this->halfedge();
    Vector3D ret = Vector3D();
    do {
      if (h->face()->isBoundary()) {
        h = h->twin()->next();
        continue;
      }

      FaceCIter f = h->face();
      vector<Vector3D> pts;
      HalfedgeCIter nh = h->next();
      do {
        VertexCIter v = nh->vertex();
        pts.push_back(v->position);
        nh = nh->next();
      } while (nh != f->halfedge());

      double area = 0.5 * cross(pts[1] - pts[0], pts[2] - pts[0]).norm();
      ret += f->normal() * area;
      h = h->twin()->next();
    } while(h != this->halfedge());
    return ret.unit();
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
    if (e0->isBoundary()) {
      return e0;
    }
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h5->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h2->twin();
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();
    EdgeIter e1 = h1->edge();
    EdgeIter e2 = h2->edge();
    EdgeIter e3 = h4->edge();
    EdgeIter e4 = h5->edge();
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    // setNeighbors(next, twin, vertex, edge, face)
    h0->setNeighbors(h5, h3, v2, e0, f0);
    h1->setNeighbors(h0, h6, v1, e1, f0);
    h2->setNeighbors(h4, h9, v2, e2, f1);
    h3->setNeighbors(h2, h0, v3, e0, f1);
    h4->setNeighbors(h3, h8, v0, e3, f1);
    h5->setNeighbors(h1, h7, v3, e4, f0);

    v0->halfedge() = h9;
    v1->halfedge() = h1;

    f0->halfedge() = h0;
    f1->halfedge() = h3;

    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
    if (e0->isBoundary()) {
      // This is the Halfedge that is on the face
      HalfedgeIter h0 = e0->halfedge();
      if (h0->isBoundary()) {
        h0 = h0->twin();
      }

      // removed h4, h5, h7, h8, v3
      HalfedgeIter h1 = h0->next();
      HalfedgeIter h2 = h1->next();
      HalfedgeIter h3 = h0->twin();
      HalfedgeIter h6 = h1->twin();
      HalfedgeIter h9 = h2->twin();
      VertexIter v0 = h0->vertex();
      VertexIter v1 = h1->vertex();
      VertexIter v2 = h2->vertex();
      FaceIter f0 = h0->face();
      FaceIter f1 = h3->face();

      // removed f3, e3, h14, h15
      VertexIter v4 = this->newVertex();
      FaceIter f2 = this->newFace();
      EdgeIter e1 = this->newEdge();
      EdgeIter e2 = this->newEdge();
      HalfedgeIter h10 = this->newHalfedge();
      HalfedgeIter h11 = this->newHalfedge();
      HalfedgeIter h12 = this->newHalfedge();
      HalfedgeIter h13 = this->newHalfedge();

      v4->position = (v0->position + v1->position) / 2.0;
      v4->halfedge() = h0;

      f2->halfedge() = h10;

      e1->halfedge() = h10;
      e2->halfedge() = h2;

      // setNeighbors(next, twin, vertex, edge, face)
      h10->setNeighbors(h11, h13, v0, e1, f2);
      h11->setNeighbors(h12, h2, v4, e2, f2);
      h12->setNeighbors(h10, h9, v2, h9->edge(), f2);
      h13->setNeighbors(h3->next(), h10, v4, e1, f1); // changed this

      h0->setNeighbors(h1, h3, v4, e0, f0);
      h2->setNeighbors(h0, h11, v2, e2, f0);
      h9->setNeighbors(h9->next(), h12, v0, h9->edge(), h9->face());

      v0->halfedge() = h10;
      h9->edge()->halfedge() = h9;

      return v4;
    }
    HalfedgeIter h0 = e0->halfedge();
    HalfedgeIter h1 = h0->next();
    HalfedgeIter h2 = h1->next();
    HalfedgeIter h3 = h0->twin();
    HalfedgeIter h4 = h3->next();
    HalfedgeIter h5 = h4->next();
    HalfedgeIter h6 = h1->twin();
    HalfedgeIter h7 = h5->twin();
    HalfedgeIter h8 = h4->twin();
    HalfedgeIter h9 = h2->twin();
    VertexIter v0 = h0->vertex();
    VertexIter v1 = h1->vertex();
    VertexIter v2 = h2->vertex();
    VertexIter v3 = h5->vertex();
    FaceIter f0 = h0->face();
    FaceIter f1 = h3->face();

    VertexIter v4 = this->newVertex();
    FaceIter f2 = this->newFace();
    FaceIter f3 = this->newFace();
    EdgeIter e1 = this->newEdge();
    EdgeIter e2 = this->newEdge();
    EdgeIter e3 = this->newEdge();
    HalfedgeIter h10 = this->newHalfedge();
    HalfedgeIter h11 = this->newHalfedge();
    HalfedgeIter h12 = this->newHalfedge();
    HalfedgeIter h13 = this->newHalfedge();
    HalfedgeIter h14 = this->newHalfedge();
    HalfedgeIter h15 = this->newHalfedge();

    v4->position = (v0->position + v1->position) / 2.0;
    v4->halfedge() = h0;

    f2->halfedge() = h10;
    f3->halfedge() = h13;

    e1->halfedge() = h10;
    e2->halfedge() = h2;
    e3->halfedge() = h4;

    // setNeighbors(next, twin, vertex, edge, face)
    h10->setNeighbors(h11, h13, v0, e1, f2);
    h11->setNeighbors(h12, h2, v4, e2, f2);
    h12->setNeighbors(h10, h9, v2, h9->edge(), f2);
    h13->setNeighbors(h14, h10, v4, e1, f3);
    h14->setNeighbors(h15, h8, v0, h8->edge(), f3);
    h15->setNeighbors(h13, h4, v3, e3, f3);

    h0->setNeighbors(h1, h3, v4, e0, f0);
    h2->setNeighbors(h0, h11, v2, e2, f0);
    h4->setNeighbors(h5, h15, v4, e3, f1);
    h8->setNeighbors(h8->next(), h14, v3, h8->edge(), h8->face());
    h9->setNeighbors(h9->next(), h12, v0, h9->edge(), h9->face());

    v0->halfedge() = h10;
    h8->edge()->halfedge() = h8;
    h9->edge()->halfedge() = h9;

    return v4;
  }



  void MeshResampler::upsample( HalfedgeMesh& mesh )
  {
    // TODO Part 6.
    // This routine should increase the number of triangles in the mesh using Loop subdivision.
    // One possible solution is to break up the method as listed below.

    // 1. Compute new positions for all the vertices in the input mesh, using the Loop subdivision rule,
    // and store them in Vertex::newPosition. At this point, we also want to mark each vertex as being
    // a vertex of the original mesh.
    
    // 2. Compute the updated vertex positions associated with edges, and store it in Edge::newPosition.
    
    // 3. Split every edge in the mesh, in any order. For future reference, we're also going to store some
    // information about which subdivide edges come from splitting an edge in the original mesh, and which edges
    // are new, by setting the flat Edge::isNew. Note that in this loop, we only want to iterate over edges of
    // the original mesh---otherwise, we'll end up splitting edges that we just split (and the loop will never end!)
    
    // 4. Flip any new edge that connects an old and new vertex.

    // 5. Copy the new vertex positions into final Vertex::position.

  }
}
