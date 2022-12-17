#include <math.h>
#include "scene.hpp"
#include "loader/skinning_loader.hpp"

using namespace cgp;


void scene_structure::initialize()
{	
	gui.display_frame = false;

	environment.camera.look_at({0, 0.4, 1.2 }, { 0,0.4,0 });
	virtual_camera = camera_around_center();
	// Create a visual frame representing the coordinate system
	global_frame.initialize(mesh_primitive_frame(), "Frame");
	floor.initialize(mesh_primitive_cylinder(1.0f,vec3(0,0,0),vec3(0,-0.02f,0),10,20,true), "floor");
	
	// The 3D character
	GLuint texture_id = mesh_drawable::default_texture;
	mesh shape;
	load_skinning_data("assets/mae_edited/", skeleton_data, rig, shape, texture_id);
	normalize_weights(rig.weight);
	correct_joints(rig.joint);
	float const scaling = 0.4f;
	for (auto& p : shape.position) p *= scaling;
		skeleton_data.scale(scaling);
	update_new_content(shape,texture_id);
	visual_data.surface_skinned.shading.phong.specular = 0;

	// body line data
	bl_data.picked.resize(skeleton_data.parent_index.size());
	bl_data.picked.fill(0);

	reskinning_needed = true;
}

void scene_structure::compute_deformation()
{	
	quaternion rot = rotation_transform::convert_axis_angle_to_quaternion(vec3(0,1,0),0.1);
	// propagate_rotation_rec(0,  rot);
	visual_data.skeleton_current.update(skinning_data.skeleton_current, skeleton_data.parent_index);

	// Compute skinning deformation
	skinning_LBS_compute(skinning_data.position_skinned, skinning_data.normal_skinned,
		skinning_data.skeleton_current, skinning_data.skeleton_rest_pose,
		skinning_data.position_rest_pose, skinning_data.normal_rest_pose,
		rig);
	visual_data.surface_skinned.update_position(skinning_data.position_skinned);
	visual_data.surface_skinned.update_normal(skinning_data.normal_skinned);

}

void scene_structure::propagate_rotation_rec(int main_parent, int current_parent, quaternion rot){
	if(current_parent == 0 && (gui.orientation_lock)) return;
	std::cout << rot << std::endl;
	buffer<affine_rt> skeleton = skinning_data.skeleton_current;
		skinning_data.skeleton_current[current_parent].translation = skeleton[main_parent].translation + rot * (skeleton[current_parent].translation - skeleton[main_parent].translation);
		skinning_data.skeleton_current[current_parent].rotation = rot * skeleton[current_parent].rotation;
	for(int k = 0; k < skeleton_data.parent_index.size(); k++){
		if(skeleton_data.parent_index[k] == current_parent){
			std::cout << k << std::endl;
			propagate_rotation_rec(main_parent, k, rot);
		}
	}
}

void scene_structure::display()
{	
	// reskinning_needed = true;
	environment.light = environment.camera.position();
	if(reskinning_needed){
		compute_deformation();
		reskinning_needed = false;
	}

	for(int k=0; k<sketch_drawable.size(); ++k)
		draw(sketch_drawable[k], environment);

	if (gui.display_frame) // conditional display of the global frame (set via the GUI)
		draw(global_frame, environment);
	draw(floor,environment);
	draw(visual_data.skeleton_current, environment); 

	if (gui.surface_skinned)   // Draw the mesh if enable
		draw(visual_data.surface_skinned, environment);
	if (gui.wireframe_skinned) // Draw the mesh wireframe if enable
		draw_wireframe(visual_data.surface_skinned, environment, { 0.5f, 0.5f, 0.5f });
}

void scene_structure::update_new_content(mesh const& shape, GLuint texture_id)
{
	visual_data.surface_skinned.clear();
	visual_data.surface_skinned.initialize(shape, "Skinned surface");
	visual_data.surface_skinned.texture = texture_id;

	visual_data.surface_rest_pose.clear();
	visual_data.surface_rest_pose.initialize(shape, "Rest pose");
	visual_data.surface_rest_pose.texture = texture_id;

	skinning_data.position_rest_pose = shape.position;
	skinning_data.position_skinned = skinning_data.position_rest_pose;
	skinning_data.normal_rest_pose = shape.normal;
	skinning_data.normal_skinned = skinning_data.normal_rest_pose;

	skinning_data.skeleton_current = skeleton_data.rest_pose_global();
	skinning_data.skeleton_rest_pose = skinning_data.skeleton_current;

	visual_data.skeleton_current.clear();
	visual_data.skeleton_current = skeleton_drawable(skinning_data.skeleton_current, skeleton_data.parent_index);

	visual_data.skeleton_rest_pose.clear();
	visual_data.skeleton_rest_pose = skeleton_drawable(skinning_data.skeleton_rest_pose, skeleton_data.parent_index);

}
// Compute the 3D projection of a position given by its screen coordinates. The normal of the plane is always orthogonal to Y.
vec3 scene_structure::unproject_vertical(camera_base const&  camera, camera_projection const& P, vec2 p_screen){

	virtual_camera.look_at(normalize(vec3(camera.position().x , 0.4, camera.position().z))*1.2f, { 0,0.4,0 }, {0,1,0});
	intersection_structure picking = picking_plane_orthogonal_to_camera(p_screen,vec3(0,0.4,0), virtual_camera, P);
	return picking.position;
}


// Compute the 3D position of a position given by its screen coordinates
static vec3 unproject(camera_base const&  camera, camera_projection const& P, vec2 p_screen)
{
	intersection_structure picking = picking_plane_orthogonal_to_camera(p_screen,vec3(0,0.4,0),camera, P);
	return picking.position;
}

void scene_structure::pick_sphere(){
	buffer<vec3> spehere_positions = {};
		for(int k = 0; k < skinning_data.skeleton_current.size(); k++){
			spehere_positions.push_back(skinning_data.skeleton_current[k].translation);
		}
		picking_structure pick = picking_spheres(inputs.mouse.position.current,spehere_positions, visual_data.skeleton_current.size_sphere, environment.camera, environment.projection);
		if(pick.active){
			bl_data.picked[pick.index] = !bl_data.picked[pick.index];
		}
		visual_data.skeleton_current.update_picked_joints(bl_data.picked, vec3(1,0,0));
}

/* an outdated and dummy way of matching the LOA using translation only */
/*
void scene_structure::dummy_match_loa(){
	int N_loa = loa.size();
	int N_body_line = bl_data.lines.size(); // incorrect
	for(int k = 0; k < N_body_line; k++){
		float s = float(k)/N_body_line;
		int loa_index =  s * N_loa;
		std::cout << s << " " << loa_index << std::endl;
		skinning_data.skeleton_current[bl_data.lines[k]].translation = loa[loa_index];
	}
	visual_data.skeleton_current.update(skinning_data.skeleton_current, skeleton_data.parent_index);
}
*/

// for later: allow body initial root lock (never rotate this one)

int scene_structure::update_body_line(){
	bl_data.lines.clear();
	buffer<int> todo; // copy of picked joints (buffer of bools)
	for(int k = 0; k < bl_data.picked.size(); k++)
		todo.push_back(bl_data.picked[k]);
	buffer<int> segment;
	for(int k = 0; k < todo.size(); k++){
		if(todo[k] == 1)
		{	
			// Find the root of the picked joint
			int root_index = k;
			int parent_index = skeleton_data.parent_index[root_index];
			// Go up the segment and find the root (root = parent not in the picked joints)
			while(parent_index != -1 && todo[parent_index] == 1){
				root_index = parent_index;
				parent_index = skeleton_data.parent_index[root_index];
			}
			buffer<int> children_of_root;
			for(int j = 0; j < skeleton_data.children_index[root_index].size(); j++){
				int child_of_root = skeleton_data.children_index[root_index][j];
				if(todo[child_of_root]) children_of_root.push_back(child_of_root);
			}
			for(int j = 0; j < children_of_root.size(); j++){
				segment.clear();
				segment.push_back(root_index); segment.push_back(children_of_root[j]);
				int valid = finish_segment_rec(todo, segment);
				if(!valid){
					std::cerr << "There was an error with the segment beginning by " << root_index << ", " << children_of_root[j] << std::endl;
					std::cerr << "Maybe the body line is not a valid line." << std::endl;
					return 0;
				}
				bl_data.lines.push_back(segment);
			}
			todo[root_index] = 0;
		}
	}
	buffer<int> treated;
	for(int k = 0; k < bl_data.lines.size(); k++)
		treated.push_back(bl_data.lines[k]);
	visual_data.skeleton_current.update_picked_joints(treated, vec3(0,1,0));
	return 1;
}

int scene_structure::finish_segment_rec(buffer<int>& todo, buffer<int>& segment){
	int leaf = segment[segment.size() - 1];
	buffer<int> children_todo;
	for(int k = 0; k < skeleton_data.children_index[leaf].size(); k++){
		int child = skeleton_data.children_index[leaf][k];
		if(todo[child]) children_todo.push_back(child);
	}
	todo[leaf] = 0;
	if(children_todo.size() > 1)
		return 0;
	if(children_todo.size() == 1){
		int next_leaf = children_todo[0];
		segment.push_back(next_leaf);
		return finish_segment_rec(todo, segment);
	}
	else{
		return 1;
	}
}

static float scalar_product(vec3 v0, vec3 v1){
	return v0.x*v1.x + v0.y*v1.y + v0.z*v1.z;
}



void scene_structure::rotate_pivot_match_loa(){
	int valid = update_body_line();
	if(!valid) return;
	
	int N_loa = loa.size();
	buffer<affine_rt> skeleton = skinning_data.skeleton_current;
	// For all the body lines
	for(int r = 0; r < bl_data.lines.size(); r++)
	{	
		buffer<int> line = bl_data.lines[r];
		std::cout << "Treating line: " << line << std::endl;
		int N_body_line = line.size();
		// Find corresponding point of the root on the LOA (Closest distance)
		int root_index = line[0];
		int closest_to_root_index = 0;
		float min_distance = norm(skeleton[root_index].translation - loa[0]);
		for(int k = 0; k < N_loa; k++){
			// std::cout << "Closest: " << closest_to_root_index << " d = " << min_distance << std::endl;
			float d = norm(skeleton[root_index].translation - loa[k]);
			// std::cout << "considering: " << k << " d = " << d << std::endl;
			if(d < min_distance){
				min_distance = d;
				closest_to_root_index = k;
			}
		}
		std::cout << "Closest to root index on loa: " << closest_to_root_index << " d = " << min_distance << std::endl;

		// Abandonned method for now
		/*
		// Move the LOA so the roots of the 2 lines are the same
		vec3 loa_translate = root.translation - loa[0];
		sketch_drawable[0].transform.translation += loa_translate;
		for(int k = 0; k < N_loa; k++){
			loa[k] += loa_translate;
		}
		*/

		// Compute the curvilinear index of each joint on the line;
		float total_length = 0.0f;
		float current_distance = 0.0f;
		buffer<float> s;
		s.clear();
		// total distance
		for(int k = 0; k < N_body_line - 1; k++){
			total_length += norm(skeleton[line[k+1]].translation - skeleton[line[k]].translation);
		}
		// curve index of each joint
		s.push_back(0.0f); // of the root
		for(int k = 1; k < N_body_line; k++){
			current_distance += norm(skeleton[line[k]].translation - skeleton[line[k-1]].translation);
			s.push_back(current_distance/total_length);
		}

		std::cout << "total length = " << total_length << std::endl;
		std::cout << "curve indexes = " << s << std::endl;

		// Determine toward which end of the loa this body line is pointing (Works if the LOA is not to far away from skeleton)
		int direction = 0;
		int N_loa_segment = 0;
		vec3 u_root_to_beginning_loa = normalize(loa[0] - skeleton[root_index].translation);
		vec3 u_root_to_end_loa = normalize(loa[N_loa - 1] - skeleton[root_index].translation);
		vec3 u_root_to_child = normalize(skeleton[line[1]].translation - skeleton[root_index].translation);

		float sp_to_beginning = scalar_product(u_root_to_child, u_root_to_beginning_loa);
		float sp_to_end = scalar_product(u_root_to_child, u_root_to_end_loa);
		
		if (float(closest_to_root_index) < float(N_loa) / 5) {
			direction = 1;
			N_loa_segment = N_loa - closest_to_root_index - 1;
		}
		if (float(closest_to_root_index) >= float(N_loa)*0.8f) {
			direction = -1;
			N_loa_segment = closest_to_root_index;
		}
		
		if (direction == 0) {
			if (sp_to_beginning >= sp_to_end) {
				direction = -1;
				N_loa_segment = closest_to_root_index;
			}
			else {
				direction = 1;
				N_loa_segment = N_loa - closest_to_root_index - 1;
			}
		}

		std::cout << "direction = " << direction << std::endl;
		std::cout << "N_loa_segment = " << N_loa_segment << "/" << N_loa << std::endl;

		// Displace LOA to fit on the root
		vec3 loa_translate = skinning_data.skeleton_current[root_index].translation - loa[closest_to_root_index];
		sketch_drawable[sketch_drawable.size()-1].transform.translation += loa_translate;
		for(int k = 0; k < N_loa; k++){
			loa[k] += loa_translate;
		}

		// Rotation matching
		for(int k = 0; k < N_body_line-1; k++){
			int pivot_index = line[k];
			affine_rt pivot = skinning_data.skeleton_current[pivot_index];
			vec3 e0;
			vec3 e_target;
			if(pivot_index == 0){ // Special treatment for the hips, we can rotate them following the local tangent of the LOA
				// We will always take the tangent direction from beggining to end of the LOA, as a simplification.
				float s_backward = float(closest_to_root_index)*0.1;
				int backward_offset = s_backward;
				int backward_index_on_loa = closest_to_root_index - backward_offset;
				float s_forward = float(N_loa - closest_to_root_index)*0.1;
				int forward_offset = s_forward;
				int forward_index_on_loa = closest_to_root_index + forward_offset;
				// std::cout << closest_to_root_index << " " << backward_index_on_loa << " " << forward_index_on_loa << std::endl;
				e_target = normalize(loa[forward_index_on_loa] - loa[backward_index_on_loa]);
				e0 = normalize(skinning_data.skeleton_current[0].rotation * vec3(0,1,0));
			}
			else{
				int child_index = line[k+1];
				vec3 child_current_position = skinning_data.skeleton_current[child_index].translation;
				vec3 pivot_current_position = pivot.translation;
				e0 = normalize(child_current_position - pivot_current_position);
				int child_loa_index = closest_to_root_index + direction * s[k+1] * N_loa_segment; 
				e_target = normalize(loa[child_loa_index] - pivot_current_position);
			}

			rotation_transform rot = rotation_transform::between_vector(e0,e_target);
			quaternion q = rot.quat();
			// std::cout << "q = " << q << std::endl;
			propagate_rotation_rec(pivot_index, pivot_index, q);
		}
		// Translation matching
		/*
		for (int k = 0; k < N_body_line ; k++) {
			skinning_data.skeleton_current[line[k]].translation = loa[closest_to_root_index + direction * s[k] * N_loa_segment];
		}
		*/
	}
}


int scene_structure::contains(buffer<int> buf, int elem){
	int N = buf.size();
	for(int k = 0; k < N; k++){
		if(buf[k] == elem) return 1;
	}
	return 0;
}

void scene_structure::mouse_click()
{
	if (inputs.mouse.click.last_action == last_mouse_cursor_action::click_right) 
	{
		// Create new stroke (curve_dynamic_drawable)
		int k_sketch = sketch_drawable.size();
		sketch_drawable.push_back(curve_dynamic_drawable());
		sketch_drawable[k_sketch].initialize("Sketch " + str(k_sketch));
		sketch_drawable[k_sketch].color = vec3(255,0,0);

		// Add the new clicked position
		vec3 p;
			if(gui.z_lock)
				p = unproject_vertical(environment.camera, environment.projection, inputs.mouse.position.current);
			else
				p = unproject(environment.camera, environment.projection, inputs.mouse.position.current);
		
		sketch_drawable[k_sketch].push_back(p);

		// Update LOA buffer line
		loa.clear();
		loa.push_back(p);
	}
	if(inputs.mouse.click.last_action == last_mouse_cursor_action::click_left){
		pick_sphere();
	}
}
void scene_structure::mouse_move()
{	
	if (inputs.mouse.click.right && !inputs.mouse.on_gui) {
		// Add the new clicked position
		int k_sketch = sketch_drawable.size()-1;
		if(k_sketch >= 0){
			vec3 p;
			if(gui.z_lock)
				p = unproject_vertical(environment.camera, environment.projection, inputs.mouse.position.current);
			else
				p = unproject(environment.camera, environment.projection, inputs.mouse.position.current);
			sketch_drawable[k_sketch].push_back(p);
			loa.push_back(p);
		}
	}
}

void body_line_data::clear(){
	picked.fill(0);
	lines.clear();
}

void scene_structure::display_gui()
{
	ImGui::Checkbox("Display frame", &gui.display_frame);
	
	// Skeleton displaying
	ImGui::Text("Surface: "); ImGui::SameLine();
	ImGui::Checkbox("Plain", &gui.surface_skinned); ImGui::SameLine();
	ImGui::Checkbox("Wireframe", &gui.wireframe_skinned);

	ImGui::Text("Skeleton: "); ImGui::SameLine();
	ImGui::Checkbox("Bones", &gui.skeleton_current_bone); ImGui::SameLine();
	ImGui::Checkbox("Frame", &gui.skeleton_current_frame); ImGui::SameLine();
	ImGui::Checkbox("Sphere", &gui.skeleton_current_sphere);

	ImGui::Spacing(); ImGui::Spacing();

	visual_data.skeleton_current.display_segments = gui.skeleton_current_bone;
	visual_data.skeleton_current.display_joint_frame = gui.skeleton_current_frame;
	visual_data.skeleton_current.display_joint_sphere = gui.skeleton_current_sphere;


	bool cancel = ImGui::Button("Cancel last stroke");
	if (cancel)
	{
		// remove last stroke
		int const N_stroke = sketch_drawable.size();
		if (N_stroke > 0) {
			sketch_drawable[N_stroke - 1].clear();
			sketch_drawable.resize(N_stroke - 1);
		}
	}
	bool reset_body_line = ImGui::Button("Reset body line");
	if (reset_body_line)
	{
		bl_data.clear();
		visual_data.skeleton_current.update_picked_joints(bl_data.picked, vec3(1,0,0));
	}
	
	bool update_bl = ImGui::Button("Update body line");
	if (update_bl)
	{
		update_body_line();
		std::cout << "Lines:" << std::endl;
		for(int l = 0; l < bl_data.lines.size(); l++)
			std::cout << bl_data.lines[l] << std::endl;
	}
	bool match_LOA = ImGui::Button("Match LOA");
	if (match_LOA)
	{
		rotate_pivot_match_loa();
		reskinning_needed = true;
	}
	bool reset_scene = ImGui::Button("Reset Scene");
	if(reset_scene){
		sketch_drawable.clear();
		bl_data.clear();
		initialize();
	}

	ImGui::Checkbox("Orientation lock", &gui.orientation_lock);
	ImGui::Checkbox("Z-Lock drawing", &gui.z_lock);
}
