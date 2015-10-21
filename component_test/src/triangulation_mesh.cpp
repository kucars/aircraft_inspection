#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Triangulation_3.h>
#include <CGAL/point_generators_d.h>
#include <CGAL/algorithm.h>
#include <CGAL/assertions.h>

#include <iostream>
#include <fstream>
#include <list>
#include <set>
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Triangulation_3<K>      Triangulation;
typedef Triangulation::Finite_vertices_iterator Finite_vertices_iterator;
typedef Triangulation::Finite_edges_iterator Finite_edges_iterator;
typedef Triangulation::Finite_facets_iterator Finite_facets_iterator;
typedef Triangulation::Finite_cells_iterator Finite_cells_iterator;
typedef Triangulation::Simplex        Simplex;
typedef Triangulation::Locate_type    Locate_type;
typedef Triangulation::Point          Point;
int main()
{
  // construction from a list of points :
  std::list<Point> L;
  L.push_front(Point(0,0,0));
  L.push_front(Point(1,0,0));
  L.push_front(Point(0,1,0));
  L.push_front(Point(0,1,1));
  
  Triangulation T(L.begin(), L.end());
  
  std::set<Simplex> simplices;
  
  Finite_vertices_iterator vit = T.finite_vertices_begin();
  simplices.insert(Simplex(vit));

  Finite_cells_iterator cit = T.finite_cells_begin();
  simplices.insert(Simplex(cit));
  
  Finite_edges_iterator eit = T.finite_edges_begin();
  simplices.insert(Simplex(*eit));
  
  Finite_facets_iterator fit = T.finite_facets_begin();
  simplices.insert(Simplex(*fit));

  for (std::set<Simplex>::iterator it = simplices.begin();it != simplices.end(); it++) 
  {
    std::cout << it->dimension() << std::endl;
  }
  /*
  typedef Triangulation::Face Face;
  typedef std::vector<Face> Faces;
  Faces edges;
  std::back_insert_iterator<Faces> out(edges);
  T.tds().incident_faces(T.infinite_vertex(), 1, out);  
  // collect faces of dimension 1 (edges) incident to the infinite vertex
  std::cout << "There are " << edges.size()<< " vertices on the convex hull." << std::endl;  
  */
  return 0;
}

