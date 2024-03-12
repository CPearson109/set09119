#include "PhysicsObject.h"

#include <glm/glm.hpp>

#include "PhysicsEngine.h"
#include "Mesh.h"
#include "Shader.h"


void PhysicsBody::Draw(const glm::mat4& viewMatrix, const glm::mat4& projectionMatrix) const
{
	m_shader->Use();
	//m_shader->SetUniform("model", ModelMatrix());
	//m_shader->SetUniform("view", viewMatrix);
	//m_shader->SetUniform("projection", projectionMatrix);
	m_shader->SetUniform("color", m_color);

	auto mvp = projectionMatrix * viewMatrix * ModelMatrix();
	m_shader->SetUniform("modelViewProjectionMatrix", mvp);
	m_shader->SetUniform("normalMatrix", transpose(inverse(viewMatrix * ModelMatrix())));
	m_mesh->DrawVertexArray();
}

glm::mat3 RigidBody::InverseInertia() {
	float invMass = 1.0f / Mass();;
	glm::vec3 size = Scale();

	// Calculating the inverse inertia tensor for a cuboid
	glm::mat3 invInertia(0.0f);
	invInertia[0][0] = 3.0f * invMass / (size.y * size.y + size.z * size.z);
	invInertia[1][1] = 3.0f * invMass / (size.x * size.x + size.z * size.z);
	invInertia[2][2] = 3.0f * invMass / (size.x * size.x + size.y * size.y);

	// If the body is not oriented along the principal axes, adjust for orientation
	glm::mat3 rotationMatrix = glm::mat3(Orientation());
	glm::mat3 invInertiaWorld = rotationMatrix * invInertia * glm::transpose(rotationMatrix);

	return invInertiaWorld;
}