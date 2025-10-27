
#include "../test.h"

#include "samplers.h"
#include "tri_mesh.h"

namespace PT
{

	BBox Triangle::bbox() const
	{
		// A3T2 / A3T3

		// TODO (PathTracer): Task 2 or 3
		// Compute the bounding box of the triangle.

		// Beware of flat/zero-volume boxes! You may need to
		// account for that here, or later on in BBox::hit.

		float eps = 1e-6f;

		Vec3 p0 = vertex_list[v0].position;
		Vec3 p1 = vertex_list[v1].position;
		Vec3 p2 = vertex_list[v2].position;

		Vec3 min, max;

		min.x = std::min({p0.x, p1.x, p2.x});
		min.y = std::min({p0.y, p1.y, p2.y});
		min.z = std::min({p0.z, p1.z, p2.z});

		max.x = std::max({p0.x, p1.x, p2.x});
		max.y = std::max({p0.y, p1.y, p2.y});
		max.z = std::max({p0.z, p1.z, p2.z});

		if (max.x - min.x < eps)
		{
			min.x -= eps * 0.5f;
			max.x += eps * 0.5f;
		}
		if (max.y - min.y < eps)
		{
			min.y -= eps * 0.5f;
			max.y += eps * 0.5f;
		}
		if (max.z - min.z < eps)
		{
			min.z -= eps * 0.5f;
			max.z += eps * 0.5f;
		}

		BBox box;

		box.min = min;
		box.max = max;
		return box;
	}

	Trace Triangle::hit(const Ray &ray) const
	{
		// A3T2

		// Each vertex contains a postion and surface normal
		Tri_Mesh_Vert v_0 = vertex_list[v0];
		Tri_Mesh_Vert v_1 = vertex_list[v1];
		Tri_Mesh_Vert v_2 = vertex_list[v2];

		// TODO (PathTracer): Task 2
		// Intersect the ray with the triangle defined by the three vertices.

		Vec3 s = ray.point - v_0.position;
		Vec3 e1 = v_1.position - v_0.position;
		Vec3 e2 = v_2.position - v_0.position;
		const Vec3 &d = ray.dir;

		Trace ret;
		ret.origin = ray.point;
		ret.hit = false;	   // was there an intersection?
		ret.distance = 0.0f;   // at what distance did the intersection occur?
		ret.position = Vec3{}; // where was the intersection?
		ret.normal = Vec3{};   // what was the surface normal at the intersection?
							   // (this should be interpolated between the three vertex normals)
		ret.uv = Vec2{};	   // What was the uv associated with the point of intersection?
							   // (this should be interpolated between the three vertex uvs)

		float denom = dot(cross(e1, d), e2);
		if (denom == 0)
		{
			return ret;
		}

		float coeff = 1.0f / denom;
		float u = -coeff * dot(cross(s, e2), d);
		float v = coeff * dot(cross(e1, d), s);
		float t = -coeff * dot(cross(s, e2), e1);

		bool hit = (u >= 0) && (v >= 0) && (u + v <= 1) && (t >= ray.dist_bounds.x) && (t <= ray.dist_bounds.y);

		if (hit)
		{
			ret.hit = true;
			ret.distance = t;
			ret.position = ray.point + t * ray.dir;
			ret.normal = v_0.normal * (1.0f - u - v) + v_1.normal * u + v_2.normal * v;
			ret.uv = v_0.uv * (1.0f - u - v) + v_1.uv * u + v_2.uv * v;
		}

		return ret;
	}

	Triangle::Triangle(Tri_Mesh_Vert *verts, uint32_t v0, uint32_t v1, uint32_t v2)
		: v0(v0), v1(v1), v2(v2), vertex_list(verts)
	{
	}

	Vec3 Triangle::sample(RNG &rng, Vec3 from) const
	{
		Tri_Mesh_Vert v_0 = vertex_list[v0];
		Tri_Mesh_Vert v_1 = vertex_list[v1];
		Tri_Mesh_Vert v_2 = vertex_list[v2];
		Samplers::Triangle sampler(v_0.position, v_1.position, v_2.position);
		Vec3 pos = sampler.sample(rng);
		return (pos - from).unit();
	}

	float Triangle::pdf(Ray wray, const Mat4 &T, const Mat4 &iT) const
	{

		Ray tray = wray;
		tray.transform(iT);

		Trace trace = hit(tray);
		if (trace.hit)
		{
			trace.transform(T, iT.T());
			Vec3 v_0 = T * vertex_list[v0].position;
			Vec3 v_1 = T * vertex_list[v1].position;
			Vec3 v_2 = T * vertex_list[v2].position;
			Samplers::Triangle sampler(v_0, v_1, v_2);
			float a = sampler.pdf(trace.position);
			float g = (trace.position - wray.point).norm_squared() / std::abs(dot(trace.normal, wray.dir));
			return a * g;
		}
		return 0.0f;
	}

	bool Triangle::operator==(const Triangle &rhs) const
	{
		if (Test::differs(vertex_list[v0].position, rhs.vertex_list[rhs.v0].position) ||
			Test::differs(vertex_list[v0].normal, rhs.vertex_list[rhs.v0].normal) ||
			Test::differs(vertex_list[v0].uv, rhs.vertex_list[rhs.v0].uv) ||
			Test::differs(vertex_list[v1].position, rhs.vertex_list[rhs.v1].position) ||
			Test::differs(vertex_list[v1].normal, rhs.vertex_list[rhs.v1].normal) ||
			Test::differs(vertex_list[v1].uv, rhs.vertex_list[rhs.v1].uv) ||
			Test::differs(vertex_list[v2].position, rhs.vertex_list[rhs.v2].position) ||
			Test::differs(vertex_list[v2].normal, rhs.vertex_list[rhs.v2].normal) ||
			Test::differs(vertex_list[v2].uv, rhs.vertex_list[rhs.v2].uv))
		{
			return false;
		}
		return true;
	}

	Tri_Mesh::Tri_Mesh(const Indexed_Mesh &mesh, bool use_bvh_) : use_bvh(use_bvh_)
	{
		for (const auto &v : mesh.vertices())
		{
			verts.push_back({v.pos, v.norm, v.uv});
		}

		const auto &idxs = mesh.indices();

		std::vector<Triangle> tris;
		for (size_t i = 0; i < idxs.size(); i += 3)
		{
			tris.push_back(Triangle(verts.data(), idxs[i], idxs[i + 1], idxs[i + 2]));
		}

		if (use_bvh)
		{
			triangle_bvh.build(std::move(tris), 4);
		}
		else
		{
			triangle_list = List<Triangle>(std::move(tris));
		}
	}

	Tri_Mesh Tri_Mesh::copy() const
	{
		Tri_Mesh ret;
		ret.verts = verts;
		ret.triangle_bvh = triangle_bvh.copy();
		ret.triangle_list = triangle_list.copy();
		ret.use_bvh = use_bvh;
		return ret;
	}

	BBox Tri_Mesh::bbox() const
	{
		if (use_bvh)
			return triangle_bvh.bbox();
		return triangle_list.bbox();
	}

	Trace Tri_Mesh::hit(const Ray &ray) const
	{
		if (use_bvh)
			return triangle_bvh.hit(ray);
		return triangle_list.hit(ray);
	}

	size_t Tri_Mesh::n_triangles() const
	{
		return use_bvh ? triangle_bvh.n_primitives() : triangle_list.n_primitives();
	}

	uint32_t Tri_Mesh::visualize(GL::Lines &lines, GL::Lines &active, uint32_t level,
								 const Mat4 &trans) const
	{
		if (use_bvh)
			return triangle_bvh.visualize(lines, active, level, trans);
		return 0u;
	}

	Vec3 Tri_Mesh::sample(RNG &rng, Vec3 from) const
	{
		if (use_bvh)
		{
			return triangle_bvh.sample(rng, from);
		}
		return triangle_list.sample(rng, from);
	}

	float Tri_Mesh::pdf(Ray ray, const Mat4 &T, const Mat4 &iT) const
	{
		if (use_bvh)
		{
			return triangle_bvh.pdf(ray, T, iT);
		}
		return triangle_list.pdf(ray, T, iT);
	}

} // namespace PT
