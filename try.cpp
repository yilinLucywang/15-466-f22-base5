//precondition: 'start' is on edge 'start.x -> start.y'
//  i.e. start.weights.z = 0
//
bool WalkMesh::cross_edge( WalkPoint const &start, WalkPoint *end_, glm::quat *rotation_ ) const {
	assert(start.weights.z == 0.0f);
	assert(start.indices.x <= vertices.size() && start.indices.y <= vertices.size() && start.indices.z <= vertices.size());
	assert(end_);
	auto &end = *end_;
	assert(rotation_);
	auto &rotation = *rotation_;
//!todo{

	//TODO: check if edge (start.indices.x, start.indices.y) has a triangle on the other side:
	//  hint: remember 'next_vertex'!
	//auto f = next_vertex.find(glm::vec2(start.indices.x, start.indices.y));
	glm::uvec2 next_edge = glm::uvec2(start.indices.y, start.indices.x);
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
		const glm::vec3 o_a = vertices[start.indices.x]; 
		const glm::vec3 o_b = vertices[start.indices.y]; 
		const glm::vec3 o_c = vertices[start.indices.z];


		end.weights = glm::vec3(start.weights.y, start.weights.x, start.weights.z);
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