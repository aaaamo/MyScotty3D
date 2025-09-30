
#include "halfedge.h"

#include <unordered_map>
#include <unordered_set>
#include <functional>
#include <iostream>

/******************************************************************
*********************** Local Operations **************************
******************************************************************/

/* Note on local operation return types:

	The local operations all return a std::optional<T> type. This is used so that your
	implementation can signify that it cannot perform an operation (i.e., because
	the resulting mesh does not have a valid representation).

	An optional can have two values: std::nullopt, or a value of the type it is
	parameterized on. In this way, it's similar to a pointer, but has two advantages:
	the value it holds need not be allocated elsewhere, and it provides an API that
	forces the user to check if it is null before using the value.

	In your implementation, if you have successfully performed the operation, you can
	simply return the required reference:

			... collapse the edge ...
			return collapsed_vertex_ref;

	And if you wish to deny the operation, you can return the null optional:

			return std::nullopt;

	Note that the stubs below all reject their duties by returning the null optional.
*/

/*
 * add_face: add a standalone face to the mesh
 *  sides: number of sides
 *  radius: distance from vertices to origin
 *
 * We provide this method as an example of how to make new halfedge mesh geometry.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::add_face(uint32_t sides, float radius)
{
	// faces with fewer than three sides are invalid, so abort the operation:
	if (sides < 3)
		return std::nullopt;

	std::vector<VertexRef> face_vertices;
	// In order to make the first edge point in the +x direction, first vertex should
	//  be at -90.0f - 0.5f * 360.0f / float(sides) degrees, so:
	float const start_angle = (-0.25f - 0.5f / float(sides)) * 2.0f * PI_F;
	for (uint32_t s = 0; s < sides; ++s)
	{
		float angle = float(s) / float(sides) * 2.0f * PI_F + start_angle;
		VertexRef v = emplace_vertex();
		v->position = radius * Vec3(std::cos(angle), std::sin(angle), 0.0f);
		face_vertices.emplace_back(v);
	}

	assert(face_vertices.size() == sides);

	// assemble the rest of the mesh parts:
	FaceRef face = emplace_face(false);	   // the face to return
	FaceRef boundary = emplace_face(true); // the boundary loop around the face

	std::vector<HalfedgeRef> face_halfedges; // will use later to set ->next pointers

	for (uint32_t s = 0; s < sides; ++s)
	{
		// will create elements for edge from a->b:
		VertexRef a = face_vertices[s];
		VertexRef b = face_vertices[(s + 1) % sides];

		// h is the edge on face:
		HalfedgeRef h = emplace_halfedge();
		// t is the twin, lies on boundary:
		HalfedgeRef t = emplace_halfedge();
		// e is the edge corresponding to h,t:
		EdgeRef e = emplace_edge(false); // false: non-sharp

		// set element data to something reasonable:
		//(most ops will do this with interpolate_data(), but no data to interpolate here)
		h->corner_uv = a->position.xy() / (2.0f * radius) + 0.5f;
		h->corner_normal = Vec3(0.0f, 0.0f, 1.0f);
		t->corner_uv = b->position.xy() / (2.0f * radius) + 0.5f;
		t->corner_normal = Vec3(0.0f, 0.0f, -1.0f);

		// thing -> halfedge pointers:
		e->halfedge = h;
		a->halfedge = h;
		if (s == 0)
			face->halfedge = h;
		if (s + 1 == sides)
			boundary->halfedge = t;

		// halfedge -> thing pointers (except 'next' -- will set that later)
		h->twin = t;
		h->vertex = a;
		h->edge = e;
		h->face = face;

		t->twin = h;
		t->vertex = b;
		t->edge = e;
		t->face = boundary;

		face_halfedges.emplace_back(h);
	}

	assert(face_halfedges.size() == sides);

	for (uint32_t s = 0; s < sides; ++s)
	{
		face_halfedges[s]->next = face_halfedges[(s + 1) % sides];
		face_halfedges[(s + 1) % sides]->twin->next = face_halfedges[s]->twin;
	}

	return face;
}

/*
 * bisect_edge: split an edge without splitting the adjacent faces
 *  e: edge to split
 *
 * returns: added vertex
 *
 * We provide this as an example for how to implement local operations.
 * (and as a useful subroutine!)
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::bisect_edge(EdgeRef e)
{
	// Phase 0: draw a picture
	//
	// before:
	//    ----h--->
	// v1 ----e--- v2
	//   <----t---
	//
	// after:
	//    --h->    --h2->
	// v1 --e-- vm --e2-- v2
	//    <-t2-    <--t--
	//

	// Phase 1: collect existing elements
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// Phase 2: Allocate new elements, set data
	VertexRef vm = emplace_vertex();
	vm->position = (v1->position + v2->position) / 2.0f;
	interpolate_data({v1, v2}, vm); // set bone_weights

	EdgeRef e2 = emplace_edge();
	e2->sharp = e->sharp; // copy sharpness flag

	HalfedgeRef h2 = emplace_halfedge();
	interpolate_data({h, h->next}, h2); // set corner_uv, corner_normal

	HalfedgeRef t2 = emplace_halfedge();
	interpolate_data({t, t->next}, t2); // set corner_uv, corner_normal

	// The following elements aren't necessary for the bisect_edge, but they are here to demonstrate phase 4
	FaceRef f_not_used = emplace_face();
	HalfedgeRef h_not_used = emplace_halfedge();

	// Phase 3: Reassign connectivity (careful about ordering so you don't overwrite values you may need later!)

	vm->halfedge = h2;

	e2->halfedge = h2;

	assert(e->halfedge == h); // unchanged

	// n.b. h remains on the same face so even if h->face->halfedge == h, no fixup needed (t, similarly)

	h2->twin = t;
	h2->next = h->next;
	h2->vertex = vm;
	h2->edge = e2;
	h2->face = h->face;

	t2->twin = h;
	t2->next = t->next;
	t2->vertex = vm;
	t2->edge = e;
	t2->face = t->face;

	h->twin = t2;
	h->next = h2;
	assert(h->vertex == v1); // unchanged
	assert(h->edge == e);	 // unchanged
	// h->face unchanged

	t->twin = h2;
	t->next = t2;
	assert(t->vertex == v2); // unchanged
	t->edge = e2;
	// t->face unchanged

	// Phase 4: Delete unused elements
	erase_face(f_not_used);
	erase_halfedge(h_not_used);

	// Phase 5: Return the correct iterator
	return vm;
}

/*
 * split_edge: split an edge and adjacent (non-boundary) faces
 *  e: edge to split
 *
 * returns: added vertex. vertex->halfedge should lie along e
 *
 * Note that when splitting the adjacent faces, the new edge
 * should connect to the vertex ccw from the ccw-most end of e
 * within the face.
 *
 * Do not split adjacent boundary faces.
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::split_edge(EdgeRef e)
{
	// A2L2 (REQUIRED): split_edge

	// collect
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	HalfedgeRef hn = h->next;
	HalfedgeRef hnn = h->next->next;
	HalfedgeRef tn = t->next;
	HalfedgeRef tnn = t->next->next;
	VertexRef v1 = h->next->vertex;
	VertexRef v2 = t->next->vertex;
	VertexRef v3 = h->next->next->vertex;
	VertexRef v4 = t->next->next->vertex;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;

	// connect
	VertexRef midpoint = emplace_vertex();
	midpoint->position = e->center();
	interpolate_data({v1, v2}, midpoint);

	EdgeRef esplit = emplace_edge();

	HalfedgeRef hsplit = emplace_halfedge();
	interpolate_data({h, hn}, hsplit);
	HalfedgeRef tsplit = emplace_halfedge();
	interpolate_data({t, tn}, tsplit);

	// connect esplit
	e->halfedge = h;
	midpoint->halfedge = hsplit;
	esplit->halfedge = hsplit;
	h->edge = e;
	hsplit->edge = esplit;
	t->edge = esplit;
	tsplit->edge = e;
	hsplit->vertex = midpoint;
	hsplit->next = hn;
	hsplit->twin = t;
	t->twin = hsplit;
	tsplit->vertex = midpoint;
	tsplit->next = tn;
	tsplit->twin = h;
	h->twin = tsplit;
	h->next = hsplit;
	t->next = tsplit;
	hsplit->next = hn;
	tsplit->next = tn;
	hsplit->face = f1;
	tsplit->face = f2;

	if (!f1->boundary)
	{
		FaceRef f1split = emplace_face();
		EdgeRef f1diag = emplace_edge();
		HalfedgeRef f1diag1 = emplace_halfedge();
		interpolate_data({h, hnn}, f1diag1);
		HalfedgeRef f1diag2 = emplace_halfedge();
		interpolate_data({hn, hsplit}, f1diag2);

		// connect f1 and f1split faces
		f1diag1->face = f1;
		f1diag2->face = f1split;
		h->face = f1;
		hsplit->face = f1split;
		hn->face = f1split;
		f1split->halfedge = hsplit;
		f1->halfedge = h;

		// connect f1diag
		f1diag->halfedge = f1diag1;
		f1diag1->edge = f1diag;
		f1diag2->edge = f1diag;
		f1diag1->vertex = midpoint;
		f1diag2->vertex = v3;
		f1diag1->twin = f1diag2;
		f1diag2->twin = f1diag1;
		f1diag1->next = hnn;
		h->next = f1diag1;
		f1diag2->next = hsplit;
		hn->next = f1diag2;
	}

	if (!f2->boundary)
	{
		FaceRef f2split = emplace_face();
		EdgeRef f2diag = emplace_edge();
		HalfedgeRef f2diag1 = emplace_halfedge();
		interpolate_data({t, tnn}, f2diag1);
		HalfedgeRef f2diag2 = emplace_halfedge();
		interpolate_data({tn, tsplit}, f2diag2);

		// connect f2 and f2split faces
		f2diag1->face = f2;
		f2diag2->face = f2split;
		t->face = f2;
		tsplit->face = f2split;
		tn->face = f2split;
		f2split->halfedge = tsplit;
		f2->halfedge = t;

		// connect f1diag
		f2diag->halfedge = f2diag1;
		f2diag1->edge = f2diag;
		f2diag2->edge = f2diag;
		f2diag1->vertex = midpoint;
		f2diag2->vertex = v4;
		f2diag1->twin = f2diag2;
		f2diag2->twin = f2diag1;
		f2diag1->next = tnn;
		t->next = f2diag1;
		f2diag2->next = tsplit;
		tn->next = f2diag2;
	}

	return midpoint;
}

/*
 * inset_vertex: divide a face into triangles by placing a vertex at f->center()
 *  f: the face to add the vertex to
 *
 * returns:
 *  std::nullopt if insetting a vertex would make mesh invalid
 *  the inset vertex otherwise
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::inset_vertex(FaceRef f)
{
	// A2Lx4 (OPTIONAL): inset vertex

	(void)f;
	return std::nullopt;
}

/* [BEVEL NOTE] Note on the beveling process:

	Each of the bevel_vertex, bevel_edge, and extrude_face functions do not represent
	a full bevel/extrude operation. Instead, they should update the _connectivity_ of
	the mesh, _not_ the positions of newly created vertices. In fact, you should set
	the positions of new vertices to be exactly the same as wherever they "started from."

	When you click on a mesh element while in bevel mode, one of those three functions
	is called. But, because you may then adjust the distance/offset of the newly
	beveled face, we need another method of updating the positions of the new vertices.

	This is where bevel_positions and extrude_positions come in: these functions are
	called repeatedly as you move your mouse, the position of which determines the
	amount / shrink parameters. These functions are also passed an array of the original
	vertex positions, stored just after the bevel/extrude call, in order starting at
	face->halfedge->vertex, and the original element normal, computed just *before* the
	bevel/extrude call.

	Finally, note that the amount, extrude, and/or shrink parameters are not relative
	values -- you should compute a particular new position from them, not a delta to
	apply.
*/

/*
 * bevel_vertex: creates a face in place of a vertex
 *  v: the vertex to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_vertex(VertexRef v)
{
	// A2Lx5 (OPTIONAL): Bevel Vertex
	//  Reminder: This function does not update the vertex positions.
	//  Remember to also fill in bevel_vertex_helper (A2Lx5h)

	uint32_t d = v->degree();

	if (d < 3)
	{
		return std::nullopt;
	}

	std::vector<HalfedgeRef> v_halfedges;
	std::vector<HalfedgeRef> v_twins;
	HalfedgeRef hh = v->halfedge;
	do
	{
		v_halfedges.emplace_back(hh);
		v_twins.emplace_back(hh->twin);
		hh = hh->twin->next;
	} while (hh != v->halfedge);

	// new collections
	std::vector<VertexRef> new_vertices{d};
	new_vertices[0] = v;
	std::vector<HalfedgeRef> new_halfedges{d};
	new_halfedges[0] = emplace_halfedge();
	for (uint32_t i = 1; i < d; i++)
	{
		new_vertices[i] = emplace_vertex();
		new_vertices[i]->position = v->position;
		interpolate_data({v}, new_vertices[i]);

		new_halfedges[i] = emplace_halfedge();
	}

	FaceRef nf = emplace_face();
	nf->halfedge = new_halfedges[0];

	for (uint32_t i = 0; i < d; i++)
	{
		uint32_t i_prev = i;
		uint32_t i_now = (i + 1) % d;
		uint32_t i_next = (i + 2) % d;

		// collect
		HalfedgeRef h = v_halfedges[i_now];
		HalfedgeRef t = v_twins[i_now];
		HalfedgeRef tn = v_halfedges[i_next];
		FaceRef f = t->face;

		// new collections
		EdgeRef ne = emplace_edge();
		HalfedgeRef nh = new_halfedges[i_now];
		HalfedgeRef nhn = new_halfedges[i_prev];
		interpolate_data({t, tn}, nh);
		HalfedgeRef nt = emplace_halfedge();
		interpolate_data({t, tn}, nt);
		VertexRef v1 = new_vertices[i_now];
		VertexRef v2 = new_vertices[i_next];

		// connect
		nh->face = nf;
		nh->edge = ne;
		nh->vertex = v2;
		nh->twin = nt;
		nh->next = nhn;

		nt->face = f;
		nt->edge = ne;
		nt->vertex = v1;
		nt->twin = nh;
		nt->next = tn;

		h->vertex = v1;
		t->next = nt;

		v2->halfedge = nh;

		ne->halfedge = nh;
	}

	return nf;
}

/*
 * bevel_edge: creates a face in place of an edge
 *  e: the edge to bevel
 *
 * returns: reference to the new face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::bevel_edge(EdgeRef e)
{
	// A2Lx6 (OPTIONAL): Bevel Edge
	//  Reminder: This function does not update the vertex positions.
	//  remember to also fill in bevel_edge_helper (A2Lx6h)

	if (e->on_boundary())
	{
		return std::nullopt;
	}

	HalfedgeRef eh = e->halfedge;
	HalfedgeRef et = eh->twin;
	VertexRef v1 = eh->vertex;
	VertexRef v2 = et->vertex;

	uint32_t d1 = v1->degree();
	uint32_t d2 = v2->degree();

	if ((d1 < 3) && (d2 < 3))
	{
		return std::nullopt;
	}

	uint32_t new_degree = d1 + d2 - 2;

	std::vector<HalfedgeRef> outgoing_halfedges;
	std::vector<HalfedgeRef> outgoing_twins;

	HalfedgeRef h1 = eh->twin->next;
	while (h1 != eh)
	{
		outgoing_halfedges.emplace_back(h1);
		outgoing_twins.emplace_back(h1->twin);
		h1 = h1->twin->next;
	}
	HalfedgeRef h2 = et->twin->next;
	while (h2 != et)
	{
		outgoing_halfedges.emplace_back(h2);
		outgoing_twins.emplace_back(h2->twin);
		h2 = h2->twin->next;
	}

	std::vector<VertexRef> new_vertices{new_degree};
	std::vector<HalfedgeRef> new_halfedges{new_degree};
	std::vector<HalfedgeRef> new_twins{new_degree};
	std::vector<EdgeRef> new_edges{new_degree};

	for (uint32_t i = 0; i < new_degree; i++)
	{
		if (i == 0)
		{
			new_vertices[i] = v1;
			new_halfedges[i] = eh;
			new_twins[i] = et;
			new_edges[i] = e;
		}
		else if (i == d1 - 1)
		{
			new_vertices[i] = v2;
			new_halfedges[i] = emplace_halfedge();
			new_twins[i] = emplace_halfedge();
			new_edges[i] = emplace_edge();
		}
		else
		{
			new_vertices[i] = emplace_vertex();
			if (i < d1 - 1)
			{
				new_vertices[i]->position = v1->position;
				interpolate_data({v1}, new_vertices[i]);
			}
			else
			{
				new_vertices[i]->position = v2->position;
				interpolate_data({v2}, new_vertices[i]);
			}
			new_halfedges[i] = emplace_halfedge();
			new_twins[i] = emplace_halfedge();
			new_edges[i] = emplace_edge();
		}
	}

	FaceRef nf = emplace_face();
	nf->halfedge = new_halfedges[0];

	for (uint32_t i = 0; i < new_degree; i++)
	{
		uint32_t i_prev = i;
		uint32_t i_now = (i + 1) % new_degree;
		uint32_t i_next = (i + 2) % new_degree;

		// collect
		HalfedgeRef h = outgoing_halfedges[i_now];
		HalfedgeRef t = outgoing_twins[i_now];
		HalfedgeRef tn = outgoing_halfedges[i_next];
		FaceRef f = t->face;

		EdgeRef ne = new_edges[i_now];
		HalfedgeRef nh = new_halfedges[i_now];
		HalfedgeRef nhn = new_halfedges[i_prev];
		interpolate_data({t, tn}, nh);
		HalfedgeRef nt = new_twins[i_now];
		interpolate_data({t, tn}, nt);
		VertexRef vnow = new_vertices[i_now];
		VertexRef vnext = new_vertices[i_next];

		// connect
		nh->face = nf;
		nh->edge = ne;
		nh->vertex = vnext;
		nh->twin = nt;
		nh->next = nhn;

		nt->face = f;
		nt->edge = ne;
		nt->vertex = vnow;
		nt->twin = nh;
		nt->next = tn;

		h->vertex = vnow;
		t->next = nt;

		vnext->halfedge = nh;

		ne->halfedge = nh;

		f->halfedge = nt;
	}

	return nf;
}

/*
 * extrude_face: creates a face inset into a face
 *  f: the face to inset
 *
 * returns: reference to the inner face
 *
 * see also [BEVEL NOTE] above.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::extrude_face(FaceRef f)
{
	// A2L4: Extrude Face
	//  Reminder: This function does not update the vertex positions.
	//  Remember to also fill in extrude_helper (A2L4h)

	if (f->boundary)
	{
		return std::nullopt;
	}

	uint32_t d = f->degree();

	// new vertices
	std::vector<VertexRef> new_vertices;

	// new edge/halfedge from vertex to new vertex
	std::vector<EdgeRef> v_nv_edges;
	std::vector<HalfedgeRef> v_nv_halfedges;
	std::vector<HalfedgeRef> v_nv_twins;

	// new edge/halfedge from new vertex to next new vertex
	std::vector<EdgeRef> nv_nvn_edges;
	std::vector<HalfedgeRef> nv_nvn_halfedges;
	std::vector<HalfedgeRef> nv_nvn_twins;

	// new side faces
	std::vector<FaceRef> new_faces;

	// emplace new vectors
	for (uint32_t i = 0; i < d; i++)
	{
		new_vertices.emplace_back(emplace_vertex());
		v_nv_edges.emplace_back(emplace_edge());
		v_nv_halfedges.emplace_back(emplace_halfedge());
		v_nv_twins.emplace_back(emplace_halfedge());
		nv_nvn_edges.emplace_back(emplace_edge());
		nv_nvn_halfedges.emplace_back(emplace_halfedge());
		nv_nvn_twins.emplace_back(emplace_halfedge());
		new_faces.emplace_back(emplace_face());
	}

	// original halfedges
	std::vector<HalfedgeRef> original_halfedges;

	HalfedgeRef h = f->halfedge;
	do
	{
		original_halfedges.emplace_back(h);
		h = h->next;
	} while (h != f->halfedge);

	for (uint32_t i = 0; i < d; i++)
	{
		uint32_t i_prev = i;
		uint32_t i_now = (i + 1) % d;
		uint32_t i_next = (i + 2) % d;

		// collect
		HalfedgeRef hnow = original_halfedges[i_now];
		HalfedgeRef tnow = hnow->twin;
		HalfedgeRef hnext = original_halfedges[i_next];
		VertexRef v = hnext->vertex;
		VertexRef nv = new_vertices[i_now];
		VertexRef nvp = new_vertices[i_prev];
		EdgeRef v_nv_edge = v_nv_edges[i_now];
		HalfedgeRef v_nv_h = v_nv_halfedges[i_now];
		HalfedgeRef v_nv_t = v_nv_twins[i_now];
		HalfedgeRef v_nv_tp = v_nv_twins[i_prev];
		EdgeRef nv_nvn_edge = nv_nvn_edges[i_now];
		HalfedgeRef nv_nvn_h = nv_nvn_halfedges[i_now];
		HalfedgeRef nv_nvn_hn = nv_nvn_halfedges[i_next];
		HalfedgeRef nv_nvn_t = nv_nvn_twins[i_now];
		FaceRef nf = new_faces[i_now];
		FaceRef nfn = new_faces[i_next];

		// nv position
		nv->position = v->position;
		interpolate_data({v}, nv);

		// connect
		nf->halfedge = hnow;

		v_nv_edge->halfedge = v_nv_h;
		nv_nvn_edge->halfedge = nv_nvn_h;

		nv->halfedge = v_nv_t;

		hnow->face = nf;
		hnow->next = v_nv_h;

		v_nv_h->face = nf;
		v_nv_h->edge = v_nv_edge;
		v_nv_h->vertex = v;
		v_nv_h->twin = v_nv_t;
		v_nv_h->next = nv_nvn_t;
		interpolate_data({hnow, hnext}, v_nv_h);

		v_nv_t->face = nfn;
		v_nv_t->edge = v_nv_edge;
		v_nv_t->vertex = nv;
		v_nv_t->twin = v_nv_h;
		v_nv_t->next = hnext;
		interpolate_data({hnow, hnext}, v_nv_t);

		nv_nvn_h->face = f;
		nv_nvn_h->edge = nv_nvn_edge;
		nv_nvn_h->vertex = nvp;
		nv_nvn_h->twin = nv_nvn_t;
		nv_nvn_h->next = nv_nvn_hn;
		interpolate_data({hnow}, nv_nvn_h);

		nv_nvn_t->face = nf;
		nv_nvn_t->edge = nv_nvn_edge;
		nv_nvn_t->vertex = nv;
		nv_nvn_t->twin = nv_nvn_h;
		nv_nvn_t->next = v_nv_tp;
		interpolate_data({tnow}, nv_nvn_t);
	}

	f->halfedge = nv_nvn_halfedges[0];

	return f;
}

/*
 * flip_edge: rotate non-boundary edge ccw inside its containing faces
 *  e: edge to flip
 *
 * if e is a boundary edge, does nothing and returns std::nullopt
 * if flipping e would create an invalid mesh, does nothing and returns std::nullopt
 *
 * otherwise returns the edge, post-rotation
 *
 * does not create or destroy mesh elements.
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::flip_edge(EdgeRef e)
{
	// A2L1: Flip Edge

	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	FaceRef f1 = h->face;
	FaceRef f2 = t->face;

	if (f1->boundary || f2->boundary)
	{
		return std::nullopt;
	}

	// collect
	VertexRef v1 = h->next->vertex;
	VertexRef v2 = t->next->vertex;
	VertexRef v3 = h->next->next->vertex;
	VertexRef v4 = t->next->next->vertex;

	// edge case: shouldn't happen with manifold input but check anyway
	if ((v1 == v2) || (v1 == v3) || (v1 == v4) || (v2 == v3) || (v2 == v4) || (v3 == v4))
	{
		return std::nullopt;
	}

	HalfedgeRef hn = h->next;
	HalfedgeRef hnn = h->next->next;
	HalfedgeRef tn = t->next;
	HalfedgeRef tnn = t->next->next;
	HalfedgeRef hp = h->next;
	while (hp->next != h)
	{
		hp = hp->next;
	}
	HalfedgeRef tp = t->next;
	while (tp->next != t)
	{
		tp = tp->next;
	}

	// edge case: faces connected at continuous edges
	if ((hp->twin == tn) || (hn->twin == tp))
	{
		return std::nullopt;
	}

	// edge case: check if v3->v4 edge already exists
	HalfedgeRef v3_start = v3->halfedge;
	HalfedgeRef check = v3_start;
	do
	{
		if (check->twin->vertex == v4)
		{
			return std::nullopt;
		}
		check = check->twin->next;

	} while (check != v3_start);

	// connect
	v1->halfedge = hn;
	v2->halfedge = tn;
	f1->halfedge = h;
	f2->halfedge = t;
	h->vertex = v4;
	t->vertex = v3;
	tp->next = hn;
	hp->next = tn;
	hn->next = t;
	tn->next = h;
	h->next = hnn;
	t->next = tnn;

	hn->face = f2;
	tn->face = f1;

	return h->edge;
}

/*
 * make_boundary: add non-boundary face to boundary
 *  face: the face to make part of the boundary
 *
 * if face ends up adjacent to other boundary faces, merge them into face
 *
 * if resulting mesh would be invalid, does nothing and returns std::nullopt
 * otherwise returns face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::make_boundary(FaceRef face)
{
	// A2Lx7: (OPTIONAL) make_boundary

	return std::nullopt; // TODO: actually write this code!
}

/*
 * dissolve_vertex: merge non-boundary faces adjacent to vertex, removing vertex
 *  v: vertex to merge around
 *
 * if merging would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_vertex(VertexRef v)
{
	// A2Lx1 (OPTIONAL): Dissolve Vertex

	return std::nullopt;
}

/*
 * dissolve_edge: merge the two faces on either side of an edge
 *  e: the edge to dissolve
 *
 * merging a boundary and non-boundary face produces a boundary face.
 *
 * if the result of the merge would be an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the merged face.
 */
std::optional<Halfedge_Mesh::FaceRef> Halfedge_Mesh::dissolve_edge(EdgeRef e)
{
	// A2Lx2 (OPTIONAL): dissolve_edge

	// Reminder: use interpolate_data() to merge corner_uv / corner_normal data

	return std::nullopt;
}

/* collapse_edge: collapse edge to a vertex at its middle
 *  e: the edge to collapse
 *
 * if collapsing the edge would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_edge(EdgeRef e)
{
	// A2L3: Collapse Edge

	// Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	//  (also works for bone_weights data on vertices!)

	// collect
	HalfedgeRef h = e->halfedge;
	HalfedgeRef t = h->twin;
	VertexRef v1 = h->vertex;
	VertexRef v2 = t->vertex;

	// boundary edge case
	if (v1->on_boundary() && v2->on_boundary())
	{
		if (!h->face->boundary && !t->face->boundary)
		{
			// edge cuts across boundaries -> result will be invalid
			return std::nullopt;
		}
	}

	// collect all vertices for v1, v2
	// if duplicate and h->next->next->vertex not in (v1, v2)
	// this new edge will have more than 2 faces on it so collapsing this edge will result in an invalid mesh

	std::vector<VertexRef> connected;

	HalfedgeRef test1 = h->twin->next;
	do
	{
		VertexRef vnow = test1->next->vertex;
		if (std::find(connected.begin(), connected.end(), vnow) != connected.end())
		{
			if ((test1->next->next->vertex != v2) && (test1->twin->next->next->vertex != v2))
			{
				return std::nullopt;
			}
		}
		else
		{
			connected.emplace_back(vnow);
		}
		test1 = test1->twin->next;
	} while (test1 != h);
	HalfedgeRef test2 = t->twin->next;
	do
	{
		VertexRef vnow = test2->next->vertex;
		if (std::find(connected.begin(), connected.end(), vnow) != connected.end())
		{
			if ((test2->next->next->vertex != v1) && (test2->twin->next->next->vertex != v1))
			{
				return std::nullopt;
			}
		}
		else
		{
			connected.emplace_back(vnow);
		}
		test2 = test2->twin->next;
	} while (test2 != t);

	// reject if size of connected is less than 2
	if (connected.size() < 2)
	{
		return std::nullopt;
	}

	// reject check done. start collapse here

	VertexRef midpoint = emplace_vertex();
	midpoint->position = e->center();
	interpolate_data({v1, v2}, midpoint);

	// collect
	HalfedgeRef hn = h->next;
	HalfedgeRef hp = hn;
	while (hp->next != h)
	{
		hp = hp->next;
	}
	HalfedgeRef tn = t->next;
	HalfedgeRef tp = tn;
	while (tp->next != t)
	{
		tp = tp->next;
	}

	// connect hp->hn, tp->tn to form a loop skipping over edge
	hp->next = hn;
	tp->next = tn;
	h->face->halfedge = hn;
	t->face->halfedge = tn;

	// move all connections to v1 and v2 to midpoint
	HalfedgeRef hnow = hn;
	midpoint->halfedge = hn;
	do
	{
		hnow->vertex = midpoint;
		hnow = hnow->twin->next;
	} while (hnow != hn);

	// delete unnecessary edges and faces
	std::vector<FaceRef> f_erase;
	std::vector<VertexRef> v_erase;
	std::vector<EdgeRef> e_erase;
	std::vector<HalfedgeRef> h_erase;

	HalfedgeRef start = hn;
	hnow = hn;
	do
	{
		// check for face with only 2 edges
		if (hnow->next->next->vertex == midpoint)
		{
			//        |     		 |
			//   tnow | hnow    tdup | hdup
			//   keep | erase  erase | keep
			//        |     		 |

			FaceRef fnow = hnow->face;
			EdgeRef enow = hnow->edge;
			EdgeRef edup = hnow->next->edge;
			HalfedgeRef tdup = hnow->next;
			HalfedgeRef tnow = hnow->twin;
			HalfedgeRef hdup = hnow->next->twin;

			// connect tnow and hdup
			tnow->twin = hdup;
			hdup->twin = tnow;

			enow->halfedge = hdup;
			hdup->edge = enow;
			tnow->edge = enow;
			tnow->vertex->halfedge = tnow;

			f_erase.emplace_back(fnow);
			e_erase.emplace_back(edup);
			h_erase.emplace_back(hnow);
			h_erase.emplace_back(tdup);

			// midpoint vertex
			if (midpoint->halfedge == hnow)
			{
				midpoint->halfedge = hdup;
			}

			if (start == hnow)
			{
				start = hdup;
			}

			// set hnow to hdup
			hnow = hdup;
		}
		hnow = hnow->twin->next;
	} while (hnow != start);

	// erase original edge
	h_erase.emplace_back(h);
	h_erase.emplace_back(t);
	e_erase.emplace_back(e);
	v_erase.emplace_back(v1);
	v_erase.emplace_back(v2);

	// erase everything
	for (auto ff : f_erase)
	{
		erase_face(ff);
	}
	for (auto vv : v_erase)
	{
		erase_vertex(vv);
	}
	for (auto ee : e_erase)
	{
		erase_edge(ee);
	}
	for (auto hh : h_erase)
	{
		erase_halfedge(hh);
	}

	return midpoint;
}

/*
 * collapse_face: collapse a face to a single vertex at its center
 *  f: the face to collapse
 *
 * if collapsing the face would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns the newly collapsed vertex
 */
std::optional<Halfedge_Mesh::VertexRef> Halfedge_Mesh::collapse_face(FaceRef f)
{
	// A2Lx3 (OPTIONAL): Collapse Face

	// Reminder: use interpolate_data() to merge corner_uv / corner_normal data on halfedges
	//  (also works for bone_weights data on vertices!)

	return std::nullopt;
}

/*
 * weld_edges: glue two boundary edges together to make one non-boundary edge
 *  e, e2: the edges to weld
 *
 * if welding the edges would result in an invalid mesh, does nothing and returns std::nullopt
 * otherwise returns e, updated to represent the newly-welded edge
 */
std::optional<Halfedge_Mesh::EdgeRef> Halfedge_Mesh::weld_edges(EdgeRef e, EdgeRef e2)
{
	// A2Lx8: Weld Edges

	// Reminder: use interpolate_data() to merge bone_weights data on vertices!

	return std::nullopt;
}

/*
 * bevel_positions: compute new positions for the vertices of a beveled vertex/edge
 *  face: the face that was created by the bevel operation
 *  start_positions: the starting positions of the vertices
 *     start_positions[i] is the starting position of face->halfedge(->next)^i
 *  direction: direction to bevel in (unit vector)
 *  distance: how far to bevel
 *
 * push each vertex from its starting position along its outgoing edge until it has
 *  moved distance `distance` in direction `direction`. If it runs out of edge to
 *  move along, you may choose to extrapolate, clamp the distance, or do something
 *  else reasonable.
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after bevel_vertex or bevel_edge.
 * (So you can assume the local topology is set up however your bevel_* functions do it.)
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::bevel_positions(FaceRef face, std::vector<Vec3> const &start_positions, Vec3 direction, float distance)
{
	// A2Lx5h / A2Lx6h (OPTIONAL): Bevel Positions Helper

	// The basic strategy here is to loop over the list of outgoing halfedges,
	// and use the preceding and next vertex position from the original mesh
	// (in the start_positions array) to compute an new vertex position.

	std::vector<Vec3> new_positions{face->degree()};

	HalfedgeRef h = face->halfedge;
	for (uint32_t i = 0; i < new_positions.size(); i++)
	{
		Vec3 pos = start_positions[i];
		VertexRef v_out = h->twin->next->next->vertex;

		Vec3 out_vec = v_out->position - pos;

		if (out_vec.norm() == 0)
		{
			return;
		}

		Vec3 edge_dir = out_vec.unit();
		float proj_len = dot(direction, edge_dir);

		float t;

		if (proj_len == 0)
		{
			// given direction is orthogonal to the outgoing edge -> would need to move inf to reach distance.
			// here I implemented a "reasonable" edge case handling where I just move the edge by the distance instead.
			t = distance;
		}
		else
		{
			t = distance / proj_len;
		}

		new_positions[i] = pos + t * edge_dir;

		h = h->next;
	}

	h = face->halfedge;
	for (uint32_t i = 0; i < new_positions.size(); i++)
	{
		h->vertex->position = new_positions[i];
		h = h->next;
	}
}

/*
 * extrude_positions: compute new positions for the vertices of an extruded face
 *  face: the face that was created by the extrude operation
 *  move: how much to translate the face
 *  shrink: amount to linearly interpolate vertices in the face toward the face's centroid
 *    shrink of zero leaves the face where it is
 *    positive shrink makes the face smaller (at shrink of 1, face is a point)
 *    negative shrink makes the face larger
 *
 * only changes vertex positions (no connectivity changes!)
 *
 * This is called repeatedly as the user interacts, just after extrude_face.
 * (So you can assume the local topology is set up however your extrude_face function does it.)
 *
 * Using extrude face in the GUI will assume a shrink of 0 to only extrude the selected face
 * Using bevel face in the GUI will allow you to shrink and increase the size of the selected face
 *
 * see also [BEVEL NOTE] above.
 */
void Halfedge_Mesh::extrude_positions(FaceRef face, Vec3 move, float shrink)
{
	// A2L4h: Extrude Positions Helper

	// General strategy:
	//  use mesh navigation to get starting positions from the surrounding faces,
	//  compute the centroid from these positions + use to shrink,
	//  offset by move

	Vec3 centroid = face->center() + move;

	HalfedgeRef h = face->halfedge;
	do
	{
		VertexRef v = h->vertex;
		v->position += move;
		v->position -= shrink * (v->position - centroid);

		h = h->next;
	} while (h != face->halfedge);
}
