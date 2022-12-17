#pragma once

#include "cgp/cgp.hpp"
#include "skeleton/skeleton.hpp"
#include "skeleton/skeleton_drawable.hpp"

#include "skinning/skinning.hpp"
// The element of the GUI that are not already stored in other structures
struct gui_parameters {
	bool display_frame = true;

	bool orientation_lock = true;
	bool z_lock = false;

	bool surface_skinned = false;
	bool wireframe_skinned = false;

	bool skeleton_current_bone = true;
	bool skeleton_current_frame = false;
	bool skeleton_current_sphere = true;

};

struct visual_shapes_parameters
{
	cgp::mesh_drawable surface_skinned; // Mesh
	cgp::mesh_drawable surface_rest_pose;
	cgp::skeleton_drawable skeleton_current; // Skeleton
	cgp::skeleton_drawable skeleton_rest_pose; // Skeleton initial position
};

struct skinning_current_data
{
	cgp::buffer<cgp::vec3> position_rest_pose;
	cgp::buffer<cgp::vec3> position_skinned;
	cgp::buffer<cgp::vec3> normal_rest_pose;
	cgp::buffer<cgp::vec3> normal_skinned;

	cgp::buffer<cgp::affine_rt> skeleton_current;
	cgp::buffer<cgp::affine_rt> skeleton_rest_pose;

};

struct body_line_data
{
	cgp::buffer<int> picked; // 0 or 1 if a joint is picked
	cgp::buffer<cgp::buffer<int>> lines; // The body lines, starting from the root until the last child
	void clear();
};

struct rotation_to_do
{
	int pivot_index;
	cgp::quaternion q;
};


// The structure of the custom scene
struct scene_structure {
	
	cgp::scene_environment_basic_camera_spherical_coords environment; // Standard environment controler
	cgp::inputs_interaction_parameters inputs; // storage for the current values of the inputs (mouse, keyboard, window dimension) that can be use for interaction purpose
	float time = 0;
	bool reskinning_needed;

	cgp::camera_around_center virtual_camera; // A virtual camera to unproject on non-orthogonal planes
	cgp::camera_projection virtual_projection;

	// Store the curve sketched on screen. 
	//   Each new stroke (continuous click+motion of the mouse) is a new element of the buffer
	cgp::buffer<cgp::curve_dynamic_drawable> sketch_drawable;
	cgp::buffer<cgp::vec3> loa;

	gui_parameters gui; 					  // Gui parameters
	cgp::mesh_drawable global_frame;          // The standard global frame

	// 3D elements of the scene
	cgp::mesh_drawable floor;

	cgp::skeleton_animation_structure skeleton_data;
	cgp::rig_structure rig;
	visual_shapes_parameters visual_data;
	skinning_current_data skinning_data;
	body_line_data bl_data;


	// ****************************** //
	// Functions
	// ****************************** //

	void initialize();  // Standard initialization to be called before the animation loop
	void display();     // The frame display to be called within the animation loop
	void display_gui(); // The display of the GUI, also called within the animation loop

	void compute_deformation();


	// Add new points in the sketch_drawable
	void mouse_move();

	// Add a new stroke in the sketch_drawable
	void mouse_click();

	// Update visual
	void update_new_content(cgp::mesh const& shape, GLuint texture_id);

	void propagate_rotation_rec(int main_parent, int parent, cgp::quaternion rot);

	// Allow to projct picking points on a plane orthogonal to Y, even if the camera is not.
	cgp::vec3 unproject_vertical(cgp::camera_base const&  camera, cgp::camera_projection const& P, cgp::vec2 p_screen);

	// Picks a sphere on the skeleton
	void pick_sphere();

	// Updata the body line based on the picked joints
	// Returns 1 if the update worked, 0 if it failed
	int update_body_line();
	int finish_segment_rec(cgp::buffer<int>& todo, cgp::buffer<int>& segment);

	// Match the chosen body line with the drawn LOA
	void rotate_pivot_match_loa();

	// Tool
	int contains(cgp::buffer<int> buf, int elem);
};


