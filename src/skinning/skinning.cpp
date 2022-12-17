#include "skinning.hpp"

namespace cgp
{
	void normalize_weights(buffer<buffer<float>>& weights)
	{
		size_t const N = weights.size();
		// std::cout << N << std::endl;
		for (size_t k = 0; k < N; ++k) {
			float s = 0.0f;
			if(weights[k].size() == 0){
					weights[k].push_back(weights[k-1]);
			}
			// std::cout << k << " : ";
			for(float w : weights[k]){ 
				s += w; 
				// std::cout << weights[k] << " ";
			}
			// std::cout << s << std::endl;
			assert_cgp_no_msg(s>1e-5f);
			for(float& w : weights[k]) w /= s;
		}
	}
	// ceci est une arnaque moldave pour corriger le mesh en cas de vertex qui n'ont pas de joint associé
	void correct_joints(buffer<buffer<int>>& joints){
		size_t const N = joints.size();
		for (size_t k = 0; k < N; ++k) {
			if(joints[k].size() == 0){
					joints[k].push_back(joints[k-1]);
			}
		}
	}


	// Linear Blend Skinning
	void skinning_LBS_compute(
		buffer<vec3>& position_skinned,  // position to deform
		buffer<vec3>& normal_skinned,    // normal to deform
		buffer<affine_rt> const& skeleton_current,    // rigid transforms for skeleton joints in current pose
		buffer<affine_rt> const& skeleton_rest_pose,  // rigid transforms of skeleton joints in rest pose
		buffer<vec3> const& position_rest_pose,       // vertex positions of the mesh in rest pose
		buffer<vec3> const& normal_rest_pose,         // normal coordinates of the mesh in rest pose
		rig_structure const& rig)                     // information of the skinning weights (joints and weights associated to a vertex)
	{
		size_t const N_vertex = position_rest_pose.size();
		size_t const N_joint = skeleton_current.size();

		// Sanity check on sizes of buffers
		assert_cgp_no_msg(position_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_skinned.size()==N_vertex);
		assert_cgp_no_msg(normal_rest_pose.size()==N_vertex);
		assert_cgp_no_msg(skeleton_rest_pose.size()==N_joint);
		assert_cgp_no_msg(rig.joint.size()==N_vertex);
		assert_cgp_no_msg(rig.weight.size()==N_vertex);
		// std::cout << N_joint << std::endl;
		// To do
		//   Compute the Linear Blend Skinning ...
		
			for(int i = 0; i < N_vertex; i++){
				vec4 p = {0,0,0,1};
				vec4 n = {0,0,0,1};
				for(int k = 0; k < rig.joint[i].size(); k++){
					// std::cout << k << std::endl;
					int j = rig.joint[i][k];
					p += rig.weight[i][k]*skeleton_current[j].matrix()*inverse(skeleton_rest_pose[j]).matrix()*vec4(position_rest_pose[i],1);
					n += rig.weight[i][k]*skeleton_current[j].matrix()*inverse(skeleton_rest_pose[j]).matrix()*vec4(normal_rest_pose[i],1);
				}
				position_skinned[i] = vec3(p.xyz());
				normal_skinned[i] = vec3(n.xyz());
			}
		
		

	}

}