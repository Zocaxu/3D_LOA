#include "skeleton_drawable.hpp"

namespace cgp
{
	skeleton_drawable::skeleton_drawable()
		:segments(), joint_frame(), joint_sphere(), data()
	{}

	skeleton_drawable::skeleton_drawable(buffer<affine_rt> const& skeleton, buffer<int> const& parent_index)
		: segments(), joint_frame(), joint_sphere(), data(skeleton)
	{
		size_t const N = parent_index.size();
		buffer<vec3> edges;
		for (size_t k = 1; k < N; ++k){
			size_t const parent = parent_index[k];
			assert_cgp_no_msg(parent>=0 && parent<N);
			edges.push_back(skeleton[k].translation);
			edges.push_back(skeleton[parent].translation);
		}
		
		segments.initialize(edges, "Bone-segment");
		joint_frame.initialize(mesh_primitive_frame(), "Skeleton-frame");
		joint_sphere.initialize(mesh_primitive_sphere(), "Joint-sphere");

		sphere_colors.resize(N);
		sphere_colors.fill(vec3(0.8,0.8,0.8));
	}

	void skeleton_drawable::clear()
	{
		segments.clear();
		joint_frame.clear();
		joint_sphere.clear();
		data.clear();
	}

	void skeleton_drawable::update(buffer<affine_rt> const& skeleton, buffer<int> const& parent_index)
	{
		data = skeleton;

		size_t const N = skeleton.size();
		buffer<vec3> edges;
		for (size_t k = 1; k < N; ++k){
			size_t const parent = parent_index[k];
			assert_cgp_no_msg(parent>=0 && parent<N);
			edges.push_back(skeleton[k].translation);
			edges.push_back(skeleton[parent].translation);
		}

		segments.update(edges);
	}

	// Argument can be a table of booleans or a list of joint indexes
	void skeleton_drawable::update_picked_joints(buffer<int> picked, vec3 color){
		sphere_colors.fill(vec3(0.8,0.8,0.8));
		if(picked.size() == data.size()){
			for(int k = 0; k < picked.size(); k++)
				if(picked[k]) sphere_colors[k] = color;
		}
		else{
			for(int k = 0; k < picked.size(); k++)
				sphere_colors[picked[k]] = color;
		}
	}
}