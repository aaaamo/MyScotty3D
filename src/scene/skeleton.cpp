#include <unordered_set>
#include "skeleton.h"
#include "test.h"
#include <iostream>

void Skeleton::Bone::compute_rotation_axes(Vec3 *x_, Vec3 *y_, Vec3 *z_) const
{
	assert(x_ && y_ && z_);
	auto &x = *x_;
	auto &y = *y_;
	auto &z = *z_;

	// y axis points in the direction of extent:
	y = extent.unit();
	// if extent is too short to normalize nicely, point along the skeleton's 'y' axis:
	if (!y.valid())
	{
		y = Vec3{0.0f, 1.0f, 0.0f};
	}

	// x gets skeleton's 'x' axis projected to be orthogonal to 'y':
	x = Vec3{1.0f, 0.0f, 0.0f};
	x = (x - dot(x, y) * y).unit();
	if (!x.valid())
	{
		// if y perfectly aligns with skeleton's 'x' axis, x, gets skeleton's z axis:
		x = Vec3{0.0f, 0.0f, 1.0f};
		x = (x - dot(x, y) * y).unit(); //(this should do nothing)
	}

	// z computed from x,y:
	z = cross(x, y);

	// x,z rotated by roll:
	float cr = std::cos(roll / 180.0f * PI_F);
	float sr = std::sin(roll / 180.0f * PI_F);
	// x = cr * x + sr * -z;
	// z = cross(x,y);
	std::tie(x, z) = std::make_pair(cr * x + sr * -z, cr * z + sr * x);
}

std::vector<Mat4> Skeleton::bind_pose() const
{
	// A4T2a: bone-to-skeleton transformations in the bind pose
	//(the bind pose does not rotate by Bone::pose)

	std::vector<Mat4> bind;
	bind.reserve(bones.size());

	// NOTE: bones is guaranteed to be ordered such that parents appear before child bones.

	for (auto const &bone : bones)
	{
		// (void)bone; // avoid complaints about unused bone
		// // placeholder -- your code should actually compute the correct transform:

		if (bone.parent == -1U)
		{
			bind.emplace_back(Mat4::translate(base));
			continue;
		}

		Bone parent_bone = bones[bone.parent];
		Mat4 parent_bind = bind[bone.parent];
		bind.emplace_back(parent_bind * Mat4::translate(parent_bone.extent));
	}

	assert(bind.size() == bones.size()); // should have a transform for every bone.
	return bind;
}

std::vector<Mat4> Skeleton::current_pose() const
{
	// A4T2a: bone-to-skeleton transformations in the current pose

	// Similar to bind_pose(), but takes rotation from Bone::pose into account.
	//  (and translation from Skeleton::base_offset!)

	// You'll probably want to write a loop similar to bind_pose().

	// Useful functions:
	// Bone::compute_rotation_axes() will tell you what axes (in local bone space) Bone::pose should rotate around.
	// Mat4::angle_axis(angle, axis) will produce a matrix that rotates angle (in degrees) around a given axis.

	std::vector<Mat4> pose;
	pose.reserve(bones.size());

	for (auto const &bone : bones)
	{
		Vec3 x;
		Vec3 y;
		Vec3 z;
		bone.compute_rotation_axes(&x, &y, &z);
		Mat4 rx = Mat4::angle_axis(bone.pose.x, x);
		Mat4 ry = Mat4::angle_axis(bone.pose.y, y);
		Mat4 rz = Mat4::angle_axis(bone.pose.z, z);

		Mat4 R = rz * ry * rx;

		if (bone.parent == -1U)
		{
			pose.emplace_back(Mat4::translate(base + base_offset) * R);
			continue;
		}

		Bone parent_bone = bones[bone.parent];
		Mat4 parent_pose = pose[bone.parent];
		pose.emplace_back(parent_pose * Mat4::translate(parent_bone.extent) * R);
	}

	return pose;
}

std::vector<Vec3> Skeleton::gradient_in_current_pose() const
{
	// A4T2b: IK gradient

	// Computes the gradient (partial derivative) of IK energy relative to each bone's Bone::pose, in the current pose.

	// The IK energy is the sum over all *enabled* handles of the squared distance from the tip of Handle::bone to Handle::target
	std::vector<Vec3> gradient(bones.size(), Vec3{0.0f, 0.0f, 0.0f});

	// TODO: loop over handles and over bones in the chain leading to the handle, accumulating gradient contributions.
	// remember bone.compute_rotation_axes() -- should be useful here, too!

	auto pose = current_pose();

	for (auto &handle : handles)
	{
		if (!handle.enabled)
		{
			continue;
		}

		Vec3 h = handle.target;
		Vec3 p = pose[handle.bone] * bones[handle.bone].extent;

		for (BoneIndex b = handle.bone; b < bones.size(); b = bones[b].parent)
		{
			const Bone &bone = bones[b];
			Mat4 xf = pose[b];

			Vec3 r = xf * Vec3{0.0f};

			Vec3 ax;
			Vec3 ay;
			Vec3 az;
			bone.compute_rotation_axes(&ax, &ay, &az);

			Mat4 rx = Mat4::angle_axis(bone.pose.x, ax);
			Mat4 ry = Mat4::angle_axis(bone.pose.y, ay);
			Mat4 rz = Mat4::angle_axis(bone.pose.z, az);

			Vec3 x = xf.rotate(ax);
			Vec3 y = (xf * rx.inverse()).rotate(ay);
			Vec3 z = (xf * rx.inverse() * ry.inverse()).rotate(az);

			gradient[b].x += dot(cross(x, p - r), p - h);
			gradient[b].y += dot(cross(y, p - r), p - h);
			gradient[b].z += dot(cross(z, p - r), p - h);
		}
	}

	assert(gradient.size() == bones.size());
	return gradient;
}

bool Skeleton::solve_ik(uint32_t steps)
{
	// A4T2b - gradient descent
	// check which handles are enabled
	// run `steps` iterations

	// call gradient_in_current_pose() to compute d loss / d pose
	// add ...

	float step_size = 1.0f;

	for (uint32_t i = 0; i < steps; i++)
	{
		auto gradients = gradient_in_current_pose();
		float grad_sum = 0.0f;
		for (uint32_t j = 0; j < bones.size(); j++)
		{
			grad_sum += gradients[j].norm_squared();
			bones[j].pose -= step_size * gradients[j];
		}
		if (grad_sum <= 1e-5f)
		{
			return true;
		};
	}

	// if at a local minimum (e.g., gradient is near-zero), return 'true'.
	// if run through all steps, return `false`.
	return false;
}

Vec3 Skeleton::closest_point_on_line_segment(Vec3 const &a, Vec3 const &b, Vec3 const &p)
{
	// A4T3: bone weight computation (closest point helper)

	// Return the closest point to 'p' on the line segment from a to b

	// Efficiency note: you can do this without any sqrt's! (no .unit() or .norm() is needed!)

	if (a == b)
	{
		return a;
	}
	float t = dot(p - a, b - a) / (b - a).norm_squared();
	t = std::clamp(t, 0.0f, 1.0f);

	return a + t * (b - a);
}

void Skeleton::assign_bone_weights(Halfedge_Mesh *mesh_) const
{
	assert(mesh_);
	auto &mesh = *mesh_;
	(void)mesh; // avoid complaints about unused mesh

	// A4T3: bone weight computation

	// visit every vertex and **set new values** in Vertex::bone_weights (don't append to old values)

	// be sure to use bone positions in the bind pose (not the current pose!)

	// you should fill in the helper closest_point_on_line_segment() before working on this function

	auto bind = bind_pose();
	uint32_t bone_num = (uint32_t)bones.size();
	for (auto &v : mesh_->vertices)
	{
		v.bone_weights.clear();
	}
	for (uint32_t i = 0; i < bone_num; i++)
	{
		if (bones[i].radius == 0.0f)
		{
			continue;
		}
		Vec3 start = bind[i] * Vec3{};
		Vec3 end = bind[i] * bones[i].extent;
		for (auto &v : mesh_->vertices)
		{
			Vec3 closest = closest_point_on_line_segment(start, end, v.position);
			float w = std::max(0.0f, bones[i].radius - (closest - v.position).norm()) / bones[i].radius;
			if (w > 0.0f)
			{
				Halfedge_Mesh::Vertex::Bone_Weight bw;
				bw.bone = i;
				bw.weight = w;
				v.bone_weights.emplace_back(bw);
			}
		}
	}
	for (auto &v : mesh_->vertices)
	{
		if (v.bone_weights.size() > 0)
		{
			uint32_t size_ = (uint32_t)v.bone_weights.size();
			float sum_ = 0.0f;
			for (uint32_t i = 0; i < size_; i++)
			{
				sum_ += v.bone_weights[i].weight;
			}
			for (uint32_t i = 0; i < size_; i++)
			{
				v.bone_weights[i].weight /= sum_;
			}
		}
	}
}

Indexed_Mesh Skeleton::skin(Halfedge_Mesh const &mesh, std::vector<Mat4> const &bind, std::vector<Mat4> const &current)
{
	assert(bind.size() == current.size());

	// A4T3: linear blend skinning

	// one approach you might take is to first compute the skinned positions (at every vertex) and normals (at every corner)
	//  then generate faces in the style of Indexed_Mesh::from_halfedge_mesh

	//---- step 1: figure out skinned positions ---

	std::unordered_map<Halfedge_Mesh::VertexCRef, Vec3> skinned_positions;
	std::unordered_map<Halfedge_Mesh::HalfedgeCRef, Vec3> skinned_normals;
	// reserve hash table space to (one hopes) avoid re-hashing:
	skinned_positions.reserve(mesh.vertices.size());
	skinned_normals.reserve(mesh.halfedges.size());

	//(you will probably want to precompute some bind-to-current transformation matrices here)
	std::vector<Mat4> bind_to_current;
	bind_to_current.reserve(bind.size());
	for (uint32_t i = 0; i < bind.size(); i++)
	{
		bind_to_current.emplace_back(current[i] * bind[i].inverse());
	}

	for (auto vi = mesh.vertices.begin(); vi != mesh.vertices.end(); ++vi)
	{
		Mat4 pos_transform = vi->bone_weights.size() > 0 ? Mat4::Zero : Mat4::I;

		for (auto bw : vi->bone_weights)
		{
			pos_transform += bind_to_current[bw.bone] * bw.weight;
		}

		Mat4 norm_transform = pos_transform.remove_translate().inverse().T();

		skinned_positions.emplace(vi, pos_transform * vi->position); // PLACEHOLDER! Replace with code that computes the position of the vertex according to vi->position and vi->bone_weights.
		// NOTE: vertices with empty bone_weights should remain in place.

		// circulate corners at this vertex:
		auto h = vi->halfedge;
		do
		{
			// NOTE: could skip if h->face->boundary, since such corners don't get emitted

			skinned_normals.emplace(h, norm_transform * h->corner_normal); // PLACEHOLDER! Replace with code that properly transforms the normal vector! Make sure that you normalize correctly.

			h = h->twin->next;
		} while (h != vi->halfedge);
	}

	//---- step 2: transform into an indexed mesh ---

	// Hint: you should be able to use the code from Indexed_Mesh::from_halfedge_mesh (SplitEdges version) pretty much verbatim, you'll just need to fill in the positions and normals.

	// Indexed_Mesh result = Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges); // PLACEHOLDER! you'll probably want to copy the SplitEdges case from this function o'er here and modify it to use skinned_positions and skinned_normals.

	std::vector<Indexed_Mesh::Vert> verts;
	std::vector<Indexed_Mesh::Index> idxs;

	for (Halfedge_Mesh::FaceCRef f = mesh.faces.begin(); f != mesh.faces.end(); f++)
	{
		if (f->boundary)
			continue;

		// every corner gets its own copy of a vertex:
		uint32_t corners_begin = static_cast<uint32_t>(verts.size());
		Halfedge_Mesh::HalfedgeCRef h = f->halfedge;
		do
		{
			Indexed_Mesh::Vert vert;
			vert.pos = skinned_positions[h->vertex];
			vert.norm = skinned_normals[h];
			vert.uv = h->corner_uv;
			vert.id = f->id;
			verts.emplace_back(vert);
			h = h->next;
		} while (h != f->halfedge);
		uint32_t corners_end = static_cast<uint32_t>(verts.size());

		// divide face into a triangle fan:
		for (size_t i = corners_begin + 1; i + 1 < corners_end; i++)
		{
			idxs.emplace_back(corners_begin);
			idxs.emplace_back(static_cast<uint32_t>(i));
			idxs.emplace_back(static_cast<uint32_t>(i + 1));
		}
	}

	Indexed_Mesh result = Indexed_Mesh(std::move(verts), std::move(idxs));

	return result;
}

void Skeleton::for_bones(const std::function<void(Bone &)> &f)
{
	for (auto &bone : bones)
	{
		f(bone);
	}
}

void Skeleton::erase_bone(BoneIndex bone)
{
	assert(bone < bones.size());
	// update indices in bones:
	for (uint32_t b = 0; b < bones.size(); ++b)
	{
		if (bones[b].parent == -1U)
			continue;
		if (bones[b].parent == bone)
		{
			assert(b > bone); // topological sort!
			// keep bone tips in the same place when deleting parent bone:
			bones[b].extent += bones[bone].extent;
			bones[b].parent = bones[bone].parent;
		}
		else if (bones[b].parent > bone)
		{
			assert(b > bones[b].parent); // topological sort!
			bones[b].parent -= 1;
		}
	}
	// erase the bone
	bones.erase(bones.begin() + bone);
	// update indices in handles (and erase any handles on this bone):
	for (uint32_t h = 0; h < handles.size(); /* later */)
	{
		if (handles[h].bone == bone)
		{
			erase_handle(h);
		}
		else if (handles[h].bone > bone)
		{
			handles[h].bone -= 1;
			++h;
		}
		else
		{
			++h;
		}
	}
}

void Skeleton::erase_handle(HandleIndex handle)
{
	assert(handle < handles.size());

	// nothing internally refers to handles by index so can just delete:
	handles.erase(handles.begin() + handle);
}

Skeleton::BoneIndex Skeleton::add_bone(BoneIndex parent, Vec3 extent)
{
	assert(parent == -1U || parent < bones.size());
	Bone bone;
	bone.extent = extent;
	bone.parent = parent;
	// all other parameters left as default.

	// slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set<uint32_t> used;
	for (auto const &b : bones)
	{
		used.emplace(b.channel_id);
	}
	while (used.count(next_bone_channel_id))
		++next_bone_channel_id;
	bone.channel_id = next_bone_channel_id++;

	// all other parameters left as default.

	BoneIndex index = BoneIndex(bones.size());
	bones.emplace_back(bone);

	return index;
}

Skeleton::HandleIndex Skeleton::add_handle(BoneIndex bone, Vec3 target)
{
	assert(bone < bones.size());
	Handle handle;
	handle.bone = bone;
	handle.target = target;
	// all other parameters left as default.

	// slightly unfortunate hack:
	//(to ensure increasing IDs within an editing session, but reset on load)
	std::unordered_set<uint32_t> used;
	for (auto const &h : handles)
	{
		used.emplace(h.channel_id);
	}
	while (used.count(next_handle_channel_id))
		++next_handle_channel_id;
	handle.channel_id = next_handle_channel_id++;

	HandleIndex index = HandleIndex(handles.size());
	handles.emplace_back(handle);

	return index;
}

Skeleton Skeleton::copy()
{
	// turns out that there aren't any fancy pointer data structures to fix up here.
	return *this;
}

void Skeleton::make_valid()
{
	for (uint32_t b = 0; b < bones.size(); ++b)
	{
		if (!(bones[b].parent == -1U || bones[b].parent < b))
		{
			warn("bones[%u].parent is %u, which is not < %u; setting to -1.", b, bones[b].parent, b);
			bones[b].parent = -1U;
		}
	}
	if (bones.empty() && !handles.empty())
	{
		warn("Have %u handles but no bones. Deleting handles.", uint32_t(handles.size()));
		handles.clear();
	}
	for (uint32_t h = 0; h < handles.size(); ++h)
	{
		if (handles[h].bone >= HandleIndex(bones.size()))
		{
			warn("handles[%u].bone is %u, which is not < bones.size(); setting to 0.", h, handles[h].bone);
			handles[h].bone = 0;
		}
	}
}

//-------------------------------------------------

Indexed_Mesh Skinned_Mesh::bind_mesh() const
{
	return Indexed_Mesh::from_halfedge_mesh(mesh, Indexed_Mesh::SplitEdges);
}

Indexed_Mesh Skinned_Mesh::posed_mesh() const
{
	return Skeleton::skin(mesh, skeleton.bind_pose(), skeleton.current_pose());
}

Skinned_Mesh Skinned_Mesh::copy()
{
	return Skinned_Mesh{mesh.copy(), skeleton.copy()};
}
