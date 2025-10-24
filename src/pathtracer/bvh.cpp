
#include "bvh.h"
#include "aggregate.h"
#include "instance.h"
#include "tri_mesh.h"
#include "test.h"

#include <stack>
#include <iostream>

namespace PT
{

	struct BVHBuildData
	{
		BVHBuildData(size_t start, size_t range, size_t dst) : start(start), range(range), node(dst)
		{
		}
		size_t start; ///< start index into the primitive array
		size_t range; ///< range of index into the primitive array
		size_t node;  ///< address to update
	};

	struct SAHBucketData
	{
		BBox bb;		  ///< bbox of all primitives
		size_t num_prims; ///< number of primitives in the bucket
	};

	template <typename Primitive>
	void BVH<Primitive>::build(std::vector<Primitive> &&prims, size_t max_leaf_size)
	{
		// A3T3 - build a bvh

		// Keep these
		nodes.clear();
		primitives = std::move(prims);

		// Construct a BVH from the given vector of primitives and maximum leaf
		// size configuration.

		// TODO
		uint32_t bin_length = 16;

		auto get_bbox = [](auto begin, auto end) -> BBox
		{
			BBox bbox;
			bbox.reset();

			for (auto i = begin; i != end; ++i)
			{
				bbox.enclose(i->bbox());
			}

			return bbox;
		};

		BBox start_bbox = get_bbox(primitives.begin(), primitives.end());

		Node start_node;
		start_node.bbox = start_bbox;
		start_node.start = 0;
		start_node.size = primitives.size();
		start_node.l = 0;
		start_node.r = 0;

		auto sort_by_axis = [&](auto begin, auto end, uint32_t axis) -> void
		{
			std::sort(
				begin,
				end,
				[&](const Primitive &a, const Primitive &b)
				{ return a.bbox().center().data[axis] < b.bbox().center().data[axis]; });
		};

		std::function<void(size_t)> divide_node = [&](size_t node_idx)
		{
			Node &node = nodes[node_idx];
			node.l;
			node.r;

			uint32_t best_axis = 0;
			Node best_l;
			Node best_r;
			float best_SAH = -1.0f;

			bool best_found = false;

			for (uint32_t axis = 0; axis < 3; axis++)
			{
				sort_by_axis(
					primitives.begin() + node.start,
					primitives.begin() + node.start + node.size,
					axis);

				float min_axis = primitives[node.start].bbox().min.data[axis];
				float max_axis = primitives[node.start + node.size - 1].bbox().max.data[axis];

				if (min_axis == max_axis)
				{
					continue;
				}

				for (uint32_t i = 1; i < bin_length; i++)
				{
					float partition = min_axis + i * (max_axis - min_axis) / bin_length;
					auto r = std::partition(
						primitives.begin() + node.start,
						primitives.begin() + node.start + node.size,
						[&](Primitive &p)
						{ return p.bbox().center().data[axis] < partition; });

					size_t l_size = std::distance(primitives.begin() + node.start, r);
					size_t r_size = node.size - l_size;

					if (l_size == 0 || r_size == 0)
					{
						continue;
					}

					BBox l_bbox = get_bbox(primitives.begin() + node.start, primitives.begin() + node.start + l_size);
					BBox r_bbox = get_bbox(primitives.begin() + node.start + l_size, primitives.begin() + node.start + node.size);

					float SAH = l_bbox.surface_area() * float(l_size) + r_bbox.surface_area() * float(r_size);

					if (SAH < best_SAH || best_SAH < 0.0f)
					{
						best_SAH = SAH;
						best_axis = axis;

						best_l.bbox = l_bbox;
						best_l.start = node.start;
						best_l.size = l_size;

						best_r.bbox = r_bbox;
						best_r.start = node.start + l_size;
						best_r.size = r_size;

						best_found = true;
					}
				}
			}

			if (best_found)
			{
				sort_by_axis(
					primitives.begin() + node.start,
					primitives.begin() + node.start + node.size,
					best_axis);

				// assign l_idx / r_idx
				size_t l_idx = nodes.size();
				best_l.l = best_l.r = l_idx;
				nodes.push_back(best_l);
				size_t r_idx = nodes.size();
				best_r.l = best_r.r = r_idx;
				nodes.push_back(best_r);

				// set node l/r
				nodes[node_idx].l = l_idx;
				nodes[node_idx].r = r_idx;

				// recursively divide node for l and r
				if (nodes[l_idx].size > max_leaf_size)
				{
					divide_node(l_idx);
				}
				if (nodes[r_idx].size > max_leaf_size)
				{
					divide_node(r_idx);
				}
			}
		};

		// BVH from start node
		nodes.push_back(start_node);
		if (nodes[0].size > max_leaf_size)
		{
			divide_node(0);
		}
	}

	template <typename Primitive>
	Trace BVH<Primitive>::hit(const Ray &ray) const
	{
		// A3T3 - traverse your BVH

		// Implement ray - BVH intersection test. A ray intersects
		// with a BVH aggregate if and only if it intersects a primitive in
		// the BVH that is not an aggregate.

		// The starter code simply iterates through all the primitives.
		// Again, remember you can use hit() on any Primitive value.

		// TODO: replace this code with a more efficient traversal:

		if (nodes.size() == 0)
		{
			return Trace{};
		}

		std::function<void(size_t, Trace &)> traverse_node = [&](size_t node_idx, Trace &closest) -> void
		{
			const Node &node_now = nodes[node_idx];

			if (node_now.is_leaf())
			{
				for (size_t i = node_now.start; i < node_now.start + node_now.size; i++)
				{
					Trace hit = primitives[i].hit(ray);
					closest = Trace::min(closest, hit);
				}
			}
			else
			{
				size_t l_idx = node_now.l;
				size_t r_idx = node_now.r;

				Vec2 l_times{ray.dist_bounds.x, closest.hit ? closest.distance : ray.dist_bounds.y};
				bool l_box_hit = nodes[l_idx].bbox.hit(ray, l_times);

				Vec2 r_times{ray.dist_bounds.x, closest.hit ? closest.distance : ray.dist_bounds.y};
				bool r_box_hit = nodes[r_idx].bbox.hit(ray, r_times);

				if (l_box_hit && r_box_hit)
				{
					size_t idx_1 = (l_times.x <= r_times.x) ? l_idx : r_idx;
					size_t idx_2 = (l_times.x <= r_times.x) ? r_idx : l_idx;
					float dist_2 = (l_times.x <= r_times.x) ? r_times.x : l_times.x;

					traverse_node(idx_1, closest);

					// if idx_1 had no hit or the hit is further than idx_2 box hit
					if (!closest.hit || (closest.hit && closest.distance > dist_2))
					{
						traverse_node(idx_2, closest);
					}
				}
				else if (l_box_hit)
				{
					traverse_node(l_idx, closest);
				}
				else if (r_box_hit)
				{
					traverse_node(r_idx, closest);
				}
			}
		};

		Trace ret;
		traverse_node(0, ret);
		return ret;
	}

	template <typename Primitive>
	BVH<Primitive>::BVH(std::vector<Primitive> &&prims, size_t max_leaf_size)
	{
		build(std::move(prims), max_leaf_size);
	}

	template <typename Primitive>
	std::vector<Primitive> BVH<Primitive>::destructure()
	{
		nodes.clear();
		return std::move(primitives);
	}

	template <typename Primitive>
	template <typename P>
	typename std::enable_if<std::is_copy_assignable_v<P>, BVH<P>>::type BVH<Primitive>::copy() const
	{
		BVH<Primitive> ret;
		ret.nodes = nodes;
		ret.primitives = primitives;
		ret.root_idx = root_idx;
		return ret;
	}

	template <typename Primitive>
	Vec3 BVH<Primitive>::sample(RNG &rng, Vec3 from) const
	{
		if (primitives.empty())
			return {};
		int32_t n = rng.integer(0, static_cast<int32_t>(primitives.size()));
		return primitives[n].sample(rng, from);
	}

	template <typename Primitive>
	float BVH<Primitive>::pdf(Ray ray, const Mat4 &T, const Mat4 &iT) const
	{
		if (primitives.empty())
			return 0.0f;
		float ret = 0.0f;
		for (auto &prim : primitives)
			ret += prim.pdf(ray, T, iT);
		return ret / primitives.size();
	}

	template <typename Primitive>
	void BVH<Primitive>::clear()
	{
		nodes.clear();
		primitives.clear();
	}

	template <typename Primitive>
	bool BVH<Primitive>::Node::is_leaf() const
	{
		// A node is a leaf if l == r, since all interior nodes must have distinct children
		return l == r;
	}

	template <typename Primitive>
	size_t BVH<Primitive>::new_node(BBox box, size_t start, size_t size, size_t l, size_t r)
	{
		Node n;
		n.bbox = box;
		n.start = start;
		n.size = size;
		n.l = l;
		n.r = r;
		nodes.push_back(n);
		return nodes.size() - 1;
	}

	template <typename Primitive>
	BBox BVH<Primitive>::bbox() const
	{
		if (nodes.empty())
			return BBox{Vec3{0.0f}, Vec3{0.0f}};
		return nodes[root_idx].bbox;
	}

	template <typename Primitive>
	size_t BVH<Primitive>::n_primitives() const
	{
		return primitives.size();
	}

	template <typename Primitive>
	uint32_t BVH<Primitive>::visualize(GL::Lines &lines, GL::Lines &active, uint32_t level,
									   const Mat4 &trans) const
	{

		std::stack<std::pair<size_t, uint32_t>> tstack;
		tstack.push({root_idx, 0u});
		uint32_t max_level = 0u;

		if (nodes.empty())
			return max_level;

		while (!tstack.empty())
		{

			auto [idx, lvl] = tstack.top();
			max_level = std::max(max_level, lvl);
			const Node &node = nodes[idx];
			tstack.pop();

			Spectrum color = lvl == level ? Spectrum(1.0f, 0.0f, 0.0f) : Spectrum(1.0f);
			GL::Lines &add = lvl == level ? active : lines;

			BBox box = node.bbox;
			box.transform(trans);
			Vec3 min = box.min, max = box.max;

			auto edge = [&](Vec3 a, Vec3 b)
			{ add.add(a, b, color); };

			edge(min, Vec3{max.x, min.y, min.z});
			edge(min, Vec3{min.x, max.y, min.z});
			edge(min, Vec3{min.x, min.y, max.z});
			edge(max, Vec3{min.x, max.y, max.z});
			edge(max, Vec3{max.x, min.y, max.z});
			edge(max, Vec3{max.x, max.y, min.z});
			edge(Vec3{min.x, max.y, min.z}, Vec3{max.x, max.y, min.z});
			edge(Vec3{min.x, max.y, min.z}, Vec3{min.x, max.y, max.z});
			edge(Vec3{min.x, min.y, max.z}, Vec3{max.x, min.y, max.z});
			edge(Vec3{min.x, min.y, max.z}, Vec3{min.x, max.y, max.z});
			edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, max.y, min.z});
			edge(Vec3{max.x, min.y, min.z}, Vec3{max.x, min.y, max.z});

			if (!node.is_leaf())
			{
				tstack.push({node.l, lvl + 1});
				tstack.push({node.r, lvl + 1});
			}
			else
			{
				for (size_t i = node.start; i < node.start + node.size; i++)
				{
					uint32_t c = primitives[i].visualize(lines, active, level - lvl, trans);
					max_level = std::max(c + lvl, max_level);
				}
			}
		}
		return max_level;
	}

	template class BVH<Triangle>;
	template class BVH<Instance>;
	template class BVH<Aggregate>;
	template BVH<Triangle> BVH<Triangle>::copy<Triangle>() const;

} // namespace PT
