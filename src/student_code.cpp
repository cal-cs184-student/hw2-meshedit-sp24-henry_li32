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
      std::vector<Vector2D> result;
      
      for (int i = 0; i < points.size()-1; i++) {
          result.push_back((1 - t) * points[i] + t * points[i + 1]);
      }
      
    return result;
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
      std::vector<Vector3D> result;

      for (int i = 0; i < points.size() - 1; i++) {
          result.push_back((1 - t) * points[i] + t * points[i + 1]);
      }

      return result;
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
      std::vector<Vector3D> cur = points;
      while (cur.size() > 1) {
          cur = BezierPatch::evaluateStep(cur, t);
      }
    return cur[0];
  }

  /**
   * Evaluates the Bezier patch at parameter (u, v)
   *
   * @param u         Scalar interpolation parameter
   * @param v         Scalar interpolation parameter (along the other axis)
   * @return Final interpolated vector
   */
  Vector3D BezierPatch::evaluate(double u, double v) const 
  {  
    // TODO Part 2.
      std::vector<Vector3D> intermediate;
      for (int i = 0; i < controlPoints.size(); i++) {
          intermediate.push_back(evaluate1D(controlPoints[i], u));
      }

    return evaluate1D(intermediate, v);
  }

  Vector3D Vertex::normal( void ) const
  {
    // TODO Part 3.
    // Returns an approximate unit normal at this vertex, computed by
    // taking the area-weighted average of the normals of neighboring
    // triangles, then normalizing.

      std::vector<VertexCIter> vertices;
      HalfedgeCIter start = this->halfedge();
      HalfedgeCIter cur = this->halfedge();
      int num_faces = 0;
      do {
          vertices.push_back(cur->next()->vertex());
          cur = cur->next();
          vertices.push_back(cur->next()->vertex());
          cur = cur->next()->twin();
          ++num_faces;
      } while (cur != start);

      Vector3D result;
      double areas = 0;
      for (int i = 0; i < num_faces; i++) {
          VertexCIter cur1 = vertices[i * 2];
          VertexCIter cur2 = vertices[i * 2+1];
          //cout << cur1->position << cur2->position <<"\n";
          Vector3D cur1_v = cur1->position - this->position;
          Vector3D cur2_v = cur2->position - cur1->position;

          double x = cur1_v.y * cur2_v.z- cur1_v.z * cur2_v.y;
          double y = -(cur1_v.x * cur2_v.z - cur1_v.z * cur2_v.x);
          double z = cur1_v.x * cur2_v.y - cur1_v.y * cur2_v.x;

          Vector3D normal = Vector3D(x,y,z);
          double area = normal.norm()/2;
          result += normal;
      }
      //cout << "\n";


      return result / result.norm();
  }

  static Vector3D crossProduct(Vector3D v1, Vector3D v2) {
      double x = v1.y * v2.z - v1.z * v2.y;
      double y = -(v1.x * v2.z - v1.z * v2.x);
      double z = v1.x * v2.y - v1.y * v2.x;
      return Vector3D(x, y, z);
  }

  EdgeIter HalfedgeMesh::flipEdge( EdgeIter e0 )
  {
    // TODO Part 4.
    // This method should flip the given edge and return an iterator to the flipped edge.
      HalfedgeIter  bc = e0->halfedge();
      HalfedgeIter cb = bc->twin();

      FaceIter left_bot = bc->face();
      FaceIter right_up = cb->face();

      if (left_bot->isBoundary() || right_up->isBoundary()) {
          return e0;
      }

      HalfedgeIter ca = bc->next();
      HalfedgeIter ab = ca->next();
      HalfedgeIter bd = cb->next();
      HalfedgeIter dc = bd->next();

      VertexIter b = bc->vertex();
      VertexIter c = cb->vertex();
      VertexIter a = ab->vertex();
      VertexIter d = dc->vertex();

      EdgeIter ca_edge = ca->edge();
      EdgeIter ab_edge = ab->edge();
      EdgeIter bd_edge = bd->edge();
      EdgeIter dc_edge = dc->edge();

      left_bot->halfedge() = bc;
      right_up->halfedge() = cb;

      bc->setNeighbors(ab, cb, d, e0, left_bot);
      cb->setNeighbors(dc, bc, a, e0, right_up);
      ca->setNeighbors(cb, ca->twin(), c, ca->edge(), right_up);
      ab->setNeighbors(bd, ab->twin(), a, ab->edge(), left_bot);
      bd->setNeighbors(bc, bd->twin(), b, bd->edge(), left_bot);
      dc->setNeighbors(ca, dc->twin(), d, dc->edge(), right_up);

      ca_edge->halfedge() = ca;
      ab_edge->halfedge() = ab;
      bd_edge->halfedge() = bd;
      dc_edge->halfedge() = dc;


      a->halfedge() = ab;
      b->halfedge() = bd;
      d->halfedge() = dc;
      c->halfedge() = ca;
      
      e0->halfedge() = bc;

     

     
    return e0;
  }

  VertexIter HalfedgeMesh::splitEdge( EdgeIter e0 )
  {
    // TODO Part 5.
    // This method should split the given edge and return an iterator to the newly inserted vertex.
    // The halfedge of this vertex should point along the edge that was split, rather than the new edges.
      HalfedgeIter  bc = e0->halfedge();
      HalfedgeIter cb = bc->twin();

      FaceIter left_up = bc->face();
      FaceIter right_up = cb->face();

      if (left_up->isBoundary() || right_up->isBoundary()) {
          return VertexIter();
      }

      HalfedgeIter ca = bc->next();
      HalfedgeIter ab = ca->next();
      HalfedgeIter bd = cb->next();
      HalfedgeIter dc = bd->next();

      VertexIter b = bc->vertex();
      VertexIter c = cb->vertex();
      VertexIter a = ab->vertex();
      VertexIter d = dc->vertex();

      EdgeIter ca_edge = ca->edge();
      EdgeIter ab_edge = ab->edge();
      EdgeIter bd_edge = bd->edge();
      EdgeIter dc_edge = dc->edge();


      VertexIter m = newVertex();
      m->position = (c->position + b->position) / 2;
      FaceIter left_down = newFace();
      FaceIter right_down = newFace();
      EdgeIter am_edge = newEdge();
      EdgeIter bm_edge = newEdge();
      EdgeIter dm_edge = newEdge();
      HalfedgeIter am = newHalfedge();
      HalfedgeIter ma = newHalfedge();
      HalfedgeIter bm = newHalfedge();
      HalfedgeIter mb = newHalfedge();
      HalfedgeIter dm = newHalfedge();
      HalfedgeIter md = newHalfedge();

      right_up->halfedge() = dc;
      left_up->halfedge() = ca;
      right_down->halfedge() = bd;
      left_down->halfedge() = ab;

      ca->setNeighbors(am, ca->twin(), c, ca_edge, left_up);
      ab->setNeighbors(bm, ab->twin(), a, ab_edge, left_down);
      bd->setNeighbors(dm, bd->twin(), b, bd_edge, right_down);
      dc->setNeighbors(cb, dc->twin(), d, dc_edge, right_up);
      bc->setNeighbors(ca, cb, m, e0, left_up);
      cb->setNeighbors(md, bc, c, e0, right_up);
      am->setNeighbors(bc, ma, a, am_edge, left_up);
      ma->setNeighbors(ab, am, m, am_edge, left_down);
      bm->setNeighbors(ma, mb, b, bm_edge, left_down);
      mb->setNeighbors(bd, bm, m, bm_edge, right_down);
      dm->setNeighbors(mb, md, d, dm_edge, right_down);
      md->setNeighbors(dc, dm, m, dm_edge, right_up);

      ca_edge->halfedge() = ca;
      ab_edge->halfedge() = ab;
      bd_edge->halfedge() = bd;
      dc_edge->halfedge() = dc;
      am_edge->halfedge() = am;
      bm_edge->halfedge() = bm;
      dm_edge->halfedge() = dm;
      e0->halfedge() = bc;

      a->halfedge() = ab;
      b->halfedge() = bd;
      d->halfedge() = dc;
      c->halfedge() = ca;
      m->halfedge() = bc;




    return m;
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
      
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
         
          std::vector<VertexIter> neighbors;
          HalfedgeIter cur = v->halfedge();
          HalfedgeIter start = v->halfedge();
          do {
              neighbors.push_back(cur->next()->vertex());
              cur = cur->twin()->next();
          } while (cur != start);

          Vector3D new_position;
          double u = 3.0 / (8 * neighbors.size());
          for (VertexIter j : neighbors) {
              new_position += u * j->position;
              
          }
          new_position += (1 - neighbors.size() * u) * v->position;
          v->newPosition = new_position;
          v->isNew = false;
      }

      vector<EdgeIter> edges;
     
      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          Vector3D new_position;
          VertexIter down = e->halfedge()->vertex();
          VertexIter up = e->halfedge()->twin()->vertex();
          VertexIter left = e->halfedge()->next()->next()->vertex();
          VertexIter right = e->halfedge()->twin()->next()->next()->vertex();
          new_position = 3.0 / 8.0 * (up->position + down->position) + 1.0 / 8.0 * (left->position + right->position);
          e->newPosition = new_position;
          e->isNew = false;
          edges.push_back(e);
      }
      

      for (EdgeIter e : edges) {
          VertexIter down = e->halfedge()->vertex();
          VertexIter up = e->halfedge()->twin()->vertex();
          VertexIter new_vertex = mesh.splitEdge(e);
          new_vertex->newPosition = e->newPosition;
          new_vertex->isNew = true;

          HalfedgeIter h1 = e->halfedge();
          HalfedgeIter h2 = e->halfedge()->twin();
          h1->next()->next()->edge()->isNew = true;
          h2->next()->edge()->isNew = true;
          
      }

      for (EdgeIter e = mesh.edgesBegin(); e != mesh.edgesEnd(); e++) {
          HalfedgeIter h1 = e->halfedge();
          HalfedgeIter h2 = e->halfedge()->twin();
          if (e->isNew && ((h1->vertex()->isNew && !h2->vertex()->isNew) || (!h1->vertex()->isNew && h2->vertex()->isNew))) {
              mesh.flipEdge(e);
          }
      }
      
      for (VertexIter v = mesh.verticesBegin(); v != mesh.verticesEnd(); v++) {
          v->position = v->newPosition;
      }

  }
}
