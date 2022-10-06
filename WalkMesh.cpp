#include "WalkMesh.hpp"

#include "read_write_chunk.hpp"

#include <glm/gtx/norm.hpp>
#include <glm/gtx/string_cast.hpp>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <string>

WalkMesh::WalkMesh(std::vector< glm::vec3 > const &vertices_, std::vector< glm::vec3 > const &normals_, std::vector< glm::uvec3 > const &triangles_)
	: vertices(vertices_), normals(normals_), triangles(triangles_) {

	//construct next_vertex map (maps each edge to the next vertex in the triangle):
	next_vertex.reserve(triangles.size()*3);
	auto do_next = [this](uint32_t a, uint32_t b, uint32_t c) {
		auto ret = next_vertex.insert(std::make_pair(glm::uvec2(a,b), c));
		assert(ret.second);
	};
	for (auto const &tri : triangles) {
		do_next(tri.x, tri.y, tri.z);
		do_next(tri.y, tri.z, tri.x);
		do_next(tri.z, tri.x, tri.y);
	}

	//DEBUG: are vertex normals consistent with geometric normals?
	for (auto const &tri : triangles) {
		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];
		glm::vec3 out = glm::normalize(glm::cross(b-a, c-a));

		float da = glm::dot(out, normals[tri.x]);
		float db = glm::dot(out, normals[tri.y]);
		float dc = glm::dot(out, normals[tri.z]);

		assert(da > 0.1f && db > 0.1f && dc > 0.1f);
	}
}

//project pt to the plane of triangle a,b,c and return the barycentric weights of the projected point:
glm::vec3 barycentric_weights(glm::vec3 const &a, glm::vec3 const &b, glm::vec3 const &c, glm::vec3 const &pt) {
	//compute barycentric weights here!
	glm::vec3 v_0 = b - a; 
	glm::vec3 v_1 = c - a;
	glm::vec3 v_2 = pt - a; 
	float d00 = glm::dot(v_0, v_0);
	float d01 = glm::dot(v_0, v_1);
	float d11 = glm::dot(v_1, v_1); 
	float d20 = glm::dot(v_2, v_0);
	float d21 = glm::dot(v_2, v_1); 
	float d = d00 * d11 - d01 * d01; 
	float v = (d11*d20 - d01*d21)/d; 
	float w = (d00*d21 - d01*d20)/d; 
	float u = 1.0f - v - w; 
	return glm::vec3(u, v, w);//source: https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
}

WalkPoint WalkMesh::nearest_walk_point(glm::vec3 const &world_point) const {
	assert(!triangles.empty() && "Cannot start on an empty walkmesh");

	WalkPoint closest;
	float closest_dis2 = std::numeric_limits< float >::infinity();

	for (auto const &tri : triangles) {
		//find closest point on triangle:

		glm::vec3 const &a = vertices[tri.x];
		glm::vec3 const &b = vertices[tri.y];
		glm::vec3 const &c = vertices[tri.z];

		//get barycentric coordinates of closest point in the plane of (a,b,c):
		glm::vec3 coords = barycentric_weights(a,b,c, world_point);

		//is that point inside the triangle?
		if (coords.x >= 0.0f && coords.y >= 0.0f && coords.z >= 0.0f) {
			//yes, point is inside triangle.
			float dis2 = glm::length2(world_point - to_world_point(WalkPoint(tri, coords)));
			if (dis2 < closest_dis2) {
				closest_dis2 = dis2;
				closest.indices = tri;
				closest.weights = coords;
			}
		} else {
			//check triangle vertices and edges:
			auto check_edge = [&world_point, &closest, &closest_dis2, this](uint32_t ai, uint32_t bi, uint32_t ci) {
				glm::vec3 const &a = vertices[ai];
				glm::vec3 const &b = vertices[bi];

				//find closest point on line segment ab:
				float along = glm::dot(world_point-a, b-a);
				float max = glm::dot(b-a, b-a);
				glm::vec3 pt;
				glm::vec3 coords;
				if (along < 0.0f) {
					pt = a;
					coords = glm::vec3(1.0f, 0.0f, 0.0f);
				} else if (along > max) {
					pt = b;
					coords = glm::vec3(0.0f, 1.0f, 0.0f);
				} else {
					float amt = along / max;
					pt = glm::mix(a, b, amt);
					coords = glm::vec3(1.0f - amt, amt, 0.0f);
				}

				float dis2 = glm::length2(world_point - pt);
				if (dis2 < closest_dis2) {
					closest_dis2 = dis2;
					closest.indices = glm::uvec3(ai, bi, ci);
					closest.weights = coords;
				}
			};
			check_edge(tri.x, tri.y, tri.z);
			check_edge(tri.y, tri.z, tri.x);
			check_edge(tri.z, tri.x, tri.y);
		}
	}
	assert(closest.indices.x < vertices.size());
	assert(closest.indices.y < vertices.size());
	assert(closest.indices.z < vertices.size());
	return closest;
}


//start at 'start.weights' on triangle 'start.indices'
// move by 'step' and...
//  ...if a wall is hit, report where + when
//  ...if no wall is hit, report final point + 1.0f
void WalkMesh::walk_in_triangle(WalkPoint const &start, glm::vec3 const &step, WalkPoint *end_, float *time_) const {
	assert(end_);
	auto &end = *end_;
	assert(time_);
	auto &time = *time_;
	end = start;
	glm::vec3 const &a = vertices[start.indices.x];
	glm::vec3 const &b = vertices[start.indices.y];
	glm::vec3 const &c = vertices[start.indices.z];
	glm::vec3 s_world = start.weights.x * a + start.weights.y * b + start.weights.z * c; 
	glm::vec3 s_move = s_world + step;
	glm::vec3 v_0 = b - a; 
	glm::vec3 v_1 = c - a;
	glm::vec3 v_2 = s_move - a; 
	float d00 = glm::dot(v_0, v_0);
	float d01 = glm::dot(v_0, v_1);
	float d11 = glm::dot(v_1, v_1); 
	float d20 = glm::dot(v_2, v_0);
	float d21 = glm::dot(v_2, v_1); 
	float d = d00 * d11 - d01 * d01; 
	float v = (d11*d20 - d01*d21)/d; 
	float w = (d00*d21 - d01*d20)/d; //source: https://gamedev.stackexchange.com/questions/23743/whats-the-most-efficient-way-to-find-barycentric-coordinates
	float u = 1.0f - v - w; 
	glm::vec3 s_new = glm::vec3(u,v,w);
	float s_u = s_new.x - start.weights.x;
	float s_v = s_new.y - start.weights.y; 
	float s_w = s_new.z - start.weights.z;
	float t_u = -1 * start.weights.x/s_u; 
	float t_v = -1 * start.weights.y/s_v; 
	float t_w = -1 * start.weights.z/s_w;
	bool is_inside = true; 
	if(v<0 || v>1){
		is_inside = false; 
	}
	if(u<0 || u>1){
		is_inside = false;
	}
	if(w<0 || w>1){
		is_inside = false;
	}
	if(is_inside){
		end = start;
		end.weights = glm::vec3(start.weights.x + s_u, start.weights.y + s_v, start.weights.z + s_w);
		time = 1.0;
	}
	else{
		std::vector<float> temp{t_u, t_v, t_w};
		sort(temp.begin(), temp.end());
		for(float x : temp){
			if(x > 0){
				time = x;
				end = start;
				end.weights = glm::vec3(start.weights.x + s_u * time, start.weights.y + s_v * time, start.weights.z + s_w * time);
				break;
			}
		}
	}
}

//apply transform
//precondition: 'start' is on edge 'start.x -> start.y'
//  i.e. start.weights.z = 0
//
bool WalkMesh::cross_edge( WalkPoint const &start, WalkPoint *end_, glm::quat *rotation_ ) const {
	std::cout<<start.indices.x<<std::endl;
	assert(start.indices.x <= vertices.size()); 
	assert(start.indices.y <= vertices.size()); 
	assert(start.indices.z <= vertices.size());
	assert(end_);
	auto &end = *end_;
	assert(rotation_);
	auto &rotation = *rotation_;
	int s_y = 0; 
	int s_x = 0; 
	int s_z = 0;
	float s_w_x = 0; 
	float s_w_y = 0;
	float s_w_z = 0; 
	if(start.weights.z == 0.0f){
		s_x = start.indices.x;
		s_y = start.indices.y;
		s_z = start.indices.z;
		s_w_x = start.weights.x; 
		s_w_y = start.weights.y; 
		s_w_z = start.weights.z; 

	}
	if(start.weights.x == 0.0f){
		s_x = start.indices.y; 
		s_y = start.indices.z; 
		s_z = start.indices.x;
		s_w_x = start.weights.y; 
		s_w_y = start.weights.z;
		s_w_z = start.weights.x;
	}
	if(start.weights.y == 0.0f){
		s_x = start.indices.z; 
		s_y = start.indices.x; 
		s_z = start.indices.y;
		s_w_x = start.weights.z; 
		s_w_y = start.weights.x; 
		s_w_z = start.weights.y;
	}
	//assert(start.weights.z == 0.0f);
//!todo{

	//TODO: check if edge (start.indices.x, start.indices.y) has a triangle on the other side:
	//  hint: remember 'next_vertex'!
	//auto f = next_vertex.find(glm::vec2(start.indices.x, start.indices.y));
	glm::uvec2 next_edge = glm::uvec2(s_y, s_x);
	auto f = next_vertex.find(next_edge);
	if (f != next_vertex.end()) {
		//This is when the next triangle exist
		//  TODO: set end's weights and indicies on that triangle:
		glm::uvec3 next_tri = glm::uvec3(next_edge.x, next_edge.y, f->second);
		//end = start;
		//glm::vec3 weights = start.weights;
		//glm::uvec3 indices = start.indices; 
		const glm::vec3 n_v_1 = vertices[next_tri.x]; 
		const glm::vec3 n_v_2 = vertices[next_tri.y]; 
		const glm::vec3 n_v_3 = vertices[next_tri.z];
		//glm::vec3 world_start = weights.x * n_v_1 + weights.y * n_v_2 + weights.z * n_v_3;;
		
		//compute barycentric weights here!
		const glm::vec3 o_a = vertices[s_x]; 
		const glm::vec3 o_b = vertices[s_y]; 
		const glm::vec3 o_c = vertices[s_z];


		end.weights = glm::vec3(s_w_y, s_w_x, s_w_z);
		end.indices = next_tri;

		//  TODO: compute rotation that takes starting triangle's normal to ending triangle's normal:
		//  hint: look up 'glm::rotation' in the glm/gtx/quaternion.hpp header
		glm::vec3 n_1 = glm::normalize(glm::cross(n_v_2 - n_v_1, n_v_3 - n_v_1)); 

		glm::vec3 n_2 = glm::normalize(glm::cross(o_b - o_a, o_c - o_a));
		rotation = glm::rotation(n_2, n_1);
		//glm::quat(1.0f, 0.0f, 0.0f, 0.0f); //identity quat (wxyz init order)
		return true;
	} else {
		//This is when the next triangle doesn't exist
		return false;
	}
//!}
}

WalkMeshes::WalkMeshes(std::string const &filename) {
	std::ifstream file(filename, std::ios::binary);

	std::vector< glm::vec3 > vertices;
	read_chunk(file, "p...", &vertices);

	std::vector< glm::vec3 > normals;
	read_chunk(file, "n...", &normals);

	std::vector< glm::uvec3 > triangles;
	read_chunk(file, "tri0", &triangles);

	std::vector< char > names;
	read_chunk(file, "str0", &names);

	struct IndexEntry {
		uint32_t name_begin, name_end;
		uint32_t vertex_begin, vertex_end;
		uint32_t triangle_begin, triangle_end;
	};

	std::vector< IndexEntry > index;
	read_chunk(file, "idxA", &index);

	if (file.peek() != EOF) {
		std::cerr << "WARNING: trailing data in walkmesh file '" << filename << "'" << std::endl;
	}

	//-----------------

	if (vertices.size() != normals.size()) {
		throw std::runtime_error("Mis-matched position and normal sizes in '" + filename + "'");
	}

	for (auto const &e : index) {
		if (!(e.name_begin <= e.name_end && e.name_end <= names.size())) {
			throw std::runtime_error("Invalid name indices in index of '" + filename + "'");
		}
		if (!(e.vertex_begin <= e.vertex_end && e.vertex_end <= vertices.size())) {
			throw std::runtime_error("Invalid vertex indices in index of '" + filename + "'");
		}
		if (!(e.triangle_begin <= e.triangle_end && e.triangle_end <= triangles.size())) {
			throw std::runtime_error("Invalid triangle indices in index of '" + filename + "'");
		}

		//copy vertices/normals:
		std::vector< glm::vec3 > wm_vertices(vertices.begin() + e.vertex_begin, vertices.begin() + e.vertex_end);
		std::vector< glm::vec3 > wm_normals(normals.begin() + e.vertex_begin, normals.begin() + e.vertex_end);

		//remap triangles:
		std::vector< glm::uvec3 > wm_triangles; wm_triangles.reserve(e.triangle_end - e.triangle_begin);
		for (uint32_t ti = e.triangle_begin; ti != e.triangle_end; ++ti) {
			if (!( (e.vertex_begin <= triangles[ti].x && triangles[ti].x < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].y && triangles[ti].y < e.vertex_end)
			    && (e.vertex_begin <= triangles[ti].z && triangles[ti].z < e.vertex_end) )) {
				throw std::runtime_error("Invalid triangle in '" + filename + "'");
			}
			wm_triangles.emplace_back(
				triangles[ti].x - e.vertex_begin,
				triangles[ti].y - e.vertex_begin,
				triangles[ti].z - e.vertex_begin
			);
		}
		
		std::string name(names.begin() + e.name_begin, names.begin() + e.name_end);

		auto ret = meshes.emplace(name, WalkMesh(wm_vertices, wm_normals, wm_triangles));
		if (!ret.second) {
			throw std::runtime_error("WalkMesh with duplicated name '" + name + "' in '" + filename + "'");
		}

	}
}

WalkMesh const &WalkMeshes::lookup(std::string const &name) const {
	auto f = meshes.find(name);
	if (f == meshes.end()) {
		throw std::runtime_error("WalkMesh with name '" + name + "' not found.");
	}
	return f->second;
}
