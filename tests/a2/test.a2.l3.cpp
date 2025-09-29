#include "test.h"
#include "geometry/halfedge.h"
#include <iostream>

static void expect_collapse(Halfedge_Mesh &mesh, Halfedge_Mesh::EdgeRef edge, Halfedge_Mesh const &after, bool should_reject = false)
{
	if (auto ret = mesh.collapse_edge(edge))
	{
		if (should_reject)
		{
			throw Test::error("collapse_edge should reject");
		}
		if (auto msg = mesh.validate())
		{
			throw Test::error("Invalid mesh: " + msg.value().second);
		}
		// check mesh shape:
		if (auto difference = Test::differs(mesh, after, Test::CheckAllBits))
		{
			throw Test::error("Resulting mesh did not match expected: " + *difference);
		}
	}
	else
	{
		if (!should_reject)
		{
			throw Test::error("collapse_edge rejected operation!");
		}
	}
}

/*
BASIC CASE

Initial mesh:
0--1\
|  | \
2--3--4
|  | /
5--6/

Collapse Edge on Edge: 2-3

After mesh:
0-----1\
 \   /  \
  \ /    \
   2------3
  / \    /
 /   \  /
4-----5/
*/
Test test_a2_l3_collapse_edge_basic_simple("a2.l3.collapse_edge.basic.simple", []()
										   {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 Vec3(-1.2f, 0.0f, 0.0f),   	 Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 3, 1}, 
		{2, 5, 6, 3}, 
		{1, 3, 4}, 
		{3, 6, 4}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		  Vec3(-1.0f, 1.0f, 0.0f), 	Vec3(1.1f, 1.0f, 0.0f),
		 			Vec3(0.0f, 0.0f, 0.0f),  			Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-1.0f, 0.0f), 		Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{2, 4, 5}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	expect_collapse(mesh, edge, after); });

/*
EDGE CASE

Initial mesh:
0--1\
|\ | \
| \2--3
|  | /
4--5/

Collapse Edge on Edge: 0-1

After mesh:
	0--\
   / \  \
  /   \  \
 /     1--2
/      | /
3------4/
*/
Test test_a2_l3_collapse_edge_edge_boundary("a2.l3.collapse_edge.edge.boundary", []()
											{
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(-1.0f, 1.1f, 0.0f), Vec3(1.1f, 1.0f, 0.0f),
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 2, 1}, 
		{0, 4, 5, 2}, 
		{1, 2, 3}, 
		{2, 5, 3}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		       Vec3(0.05f, 1.05f, 0.0f), 
		                         Vec3(1.2f, 0.0f, 0.0f),  Vec3(2.3f, 0.0f, 0.0f),
		Vec3(-1.4f,-0.7f, 0.0f), Vec3(1.5f, -1.0f, 0.0f)
	}, {
		{0, 1, 2}, 
		{0, 3, 4, 1}, 
		{1, 4, 2}
	});

	expect_collapse(mesh, edge, after); });

Test test_a2_l3_collapse_edge_tetrahedron("a2.l3.collapse_edge.tetrahedron", []()
										  {
	
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.5f, 1.0f, 0.0f),
		Vec3(0.5f, 0.5f, 1.0f),
	}, {
		{0,1,2},
		{0,3,1},
		{0,2,3},
		{1,3,2}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.5f, 0.0f, 0.0f),
		Vec3(0.5f, 1.0f, 0.0f),
		Vec3(0.5f, 0.5f, 1.0f),
	}, {
		{0,1,2},
		{0,2,1}
	});

	expect_collapse(mesh, edge, after); });

Test test_a2_l3_collapse_edge_bipyramid("a2.l3.collapse_edge.bipyramid", []()
										{
	
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.5f, 1.0f, 0.0f),
		Vec3(0.5f, 0.5f, 1.0f),
		Vec3(0.5f, 0.5f, -1.0f)
	}, {
		{0,3,1},
		{0,2,3},
		{1,3,2},
		{0,1,4},
		{0,4,2},
		{1,2,4},
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	expect_collapse(mesh, edge, mesh, true); });

Test test_a2_l3_collapse_edge_tetracylinder("a2.l3.collapse_edge.tetracylinder", []()
											{
	
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.5f, 1.0f, 0.0f),
		Vec3(0.0f, 0.0f, 1.0f),
		Vec3(1.0f, 0.0f, 1.0f),
		Vec3(0.5f, 1.0f, 1.0f),
		Vec3(0.5f, 0.5f, 2.0f),
	}, {
		{3,6,4},
		{3,5,6},
		{4,6,5},
		{0,1,2},
		{0,3,4,1},
		{1,4,5,2},
		{2,5,3,0},
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->edge;

	expect_collapse(mesh, edge, mesh, true); });

Test test_a2_l3_collapse_edge_case1("a2.l3.collapse_edge.case1", []()
									{
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(2.0f, 0.0f, 0.0f),
		Vec3(0.0f, 1.0f, 0.0f),
		Vec3(1.0f, 0.5f, 0.0f),
		Vec3(2.0f, 1.0f, 0.0f),
		Vec3(1.0f, 1.5f, 0.0f),
		Vec3(0.0f, 2.0f, 0.0f),
		Vec3(1.0f, 2.0f, 0.0f),
		Vec3(2.0f, 2.0f, 0.0f),
	}, {
		{0,1,4,3},
		{1,2,5,4},
		{3,4,6},
		{4,5,6},
		{3,6,8,7},
		{6,5,9,8}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->next->twin->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(2.0f, 0.0f, 0.0f),
		Vec3(0.0f, 1.0f, 0.0f),
		Vec3(1.0f, 1.0f, 0.0f),
		Vec3(2.0f, 1.0f, 0.0f),
		Vec3(0.0f, 2.0f, 0.0f),
		Vec3(1.0f, 2.0f, 0.0f),
		Vec3(2.0f, 2.0f, 0.0f),
	}, {
		{0,1,4,3},
		{1,2,5,4},
		{3,4,7,6},
		{4,5,8,7},
	});

	expect_collapse(mesh, edge, after); });

Test test_a2_l3_collapse_edge_reject_boundary("a2.l3.collapse_edge.reject.boundary", []()
											  {
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(2.0f, 0.0f, 0.0f),
		Vec3(0.0f, 2.0f, 0.0f),
		Vec3(1.0f, 2.0f, 0.0f),
		Vec3(2.0f, 2.0f, 0.0f),
	}, {
		{0,1,4,3},
		{1,2,5,4},
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	expect_collapse(mesh, edge, mesh, true); });

Test test_a2_l3_collapse_edge_case2("a2.l3.collapse_edge.case2", []()
									{
	Halfedge_Mesh mesh = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.0f, 1.0f, 0.0f),
		Vec3(1.0f, 1.0f, 0.0f),
	}, {
		{0,2,3}, {0,3,1}, {0,1,3,2}
	});

	Halfedge_Mesh::EdgeRef edge = mesh.halfedges.begin()->next->edge;

	Halfedge_Mesh after = Halfedge_Mesh::from_indexed_faces({
		Vec3(0.0f, 0.0f, 0.0f),
		Vec3(1.0f, 0.0f, 0.0f),
		Vec3(0.5f, 1.0f, 0.0f),
	}, {
		{0,1,2}, {0,2,1}
	});

	expect_collapse(mesh, edge, after); });