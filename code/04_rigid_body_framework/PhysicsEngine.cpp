#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "glm/gtx/orthonormalize.hpp"
#include "glm/gtx/matrix_cross_product.hpp"
#include <glm/gtc/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

using namespace glm;
using namespace std;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{

	vel += impulse / mass;

	vel += accel * dt;

	pos += vel * dt;

}

void CheckForCollision(RigidBody& rigidBody, float groundHeight, std::vector<vec3>& contactPoints) {

	auto& meshData = rigidBody.GetMesh()->Data();
	auto modelMatrix = rigidBody.ModelMatrix();

	for (size_t i = 0; i < meshData.positions.data.size(); ++i) {
		vec4 worldPosition = modelMatrix * vec4(meshData.positions.data[i], 1.0f); // Transform to world space

		if (worldPosition.y <= groundHeight) {
			contactPoints.push_back(vec3(worldPosition));
		}
	}
}

void ResolveCollision(RigidBody& rigidBody, const std::vector<vec3>& contactPoints, float groundHeight) {
	float coefficientOfRestitution = 0.2f;
	float dampingFactor = 0.9f;

	if (contactPoints.empty()) return;

	vec3 avgContactPoint(0.0f);
	for (const auto& point : contactPoints) {
		avgContactPoint += point;
	}
	avgContactPoint /= contactPoints.size();

	float penetrationDepth = groundHeight - avgContactPoint.y;
	vec3 correction = vec3(0.0f, penetrationDepth + 0.01, 0.0f); // Add a small offset to ensure the object is above the ground
	rigidBody.SetPosition(rigidBody.Position() + correction);

	vec3 velocity = rigidBody.Velocity();
	velocity.y = -velocity.y * coefficientOfRestitution; // Reflect the y component of velocity

	// Apply damping to horizontal components to simulate friction
	velocity.x *= dampingFactor;
	velocity.z *= dampingFactor;

	rigidBody.SetVelocity(velocity);

	// Adjust angular velocity based on the collision
	if (contactPoints.size() > 1) {
		vec3 angularVelocity = rigidBody.AngularVelocity();

		rigidBody.SetAngularVelocity(angularVelocity);
	}
}


void UpdateRigidBodyRotation(RigidBody& rb, float dt) {
	vec3 angVel = rb.AngularVelocity();
	quat currentOrientation = rb.Orientation();

	// Convert angular velocity to a quaternion
	quat deltaOrientation = quat(0, angVel.x * dt, angVel.y * dt, angVel.z * dt);
	deltaOrientation *= currentOrientation;

	currentOrientation += 0.5f * deltaOrientation;
	currentOrientation = normalize(currentOrientation); // Keep the quaternion normalized

	// Convert the quaternion to a 4x4 matrix
	mat4 rotationMatrix = mat4_cast(currentOrientation);

	// Pass the matrix to SetOrientation
	rb.SetOrientation(rotationMatrix);
}


// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb)
{
	// Get a few meshes/shaders from the databases
	auto defaultShader = shaderDb.Get("default");
	auto particleMesh = meshDb.Get("tetra");
	auto groundMesh = meshDb.Get("plane");

	meshDb.Add("cube", Mesh(MeshDataFromWavefrontObj("resources/models/cube.obj")));
	meshDb.Add("sphere", Mesh(MeshDataFromWavefrontObj("resources/models/sphere.obj")));
	meshDb.Add("cone", Mesh(MeshDataFromWavefrontObj("resources/models/cone.obj")));
	auto mesh = meshDb.Get("cube");

	// Initialise ground
	ground.SetMesh(groundMesh);
	ground.SetShader(defaultShader);
	ground.SetScale(vec3(10.0f));

	for (int i = 0; i < 1; ++i) {
		RigidBody particle;
		particle.SetMesh(mesh);
		particle.SetShader(defaultShader);
		particle.SetColor(vec4(1.0f, 0.0f, 0.0f, 1.0f));
		particle.SetPosition(vec3(0.0f, 5.0f, 0.0f));
		particle.SetScale(vec3(1.0f, 1.0f, 1.0f));
		particle.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
		particle.SetAngularVelocity(vec3(1.0f, 0.0f, 0.0f));
		particle.SetMass(1.0f);
		particles.push_back(particle);
	}

	camera = Camera(vec3(0, 2.5, 10));

}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{

	int groundHeight = 0;
	auto coefficientOfRestitution = 0.9f; // This can be adjusted based on the materials involved
	vector<vec3> contactPoints; // This will store the contact points for each collision

	for (auto& particle : particles) {
		vec3 p = particle.Position();
		vec3 v = particle.Velocity();
		vec3 acceleration = GRAVITY;
		vec3 impulse(0.0f);

		// First, handle the collision detection
		contactPoints.clear(); // Clear previous contact points
		CheckForCollision(particle, groundHeight, contactPoints);

		// If there are contact points, handle the collision response
		if (!contactPoints.empty()) {


			ResolveCollision(particle, contactPoints, groundHeight);

			p = particle.Position();
			v = particle.Velocity();
		}
		else {
			// If no collision, proceed with the regular update
			vec3 totalAcceleration = acceleration; // Only external force is gravity

			// Update position and velocity using the Symplectic Euler method
			SymplecticEuler(p, v, particle.Mass(), totalAcceleration, impulse, deltaTime);
		}

		// Update the rigid body's rotation
		UpdateRigidBodyRotation(particle, deltaTime);

		// Set the updated position and velocity
		particle.SetPosition(p);
		particle.SetVelocity(v);
	}
}




// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (auto& particle : particles) {
		particle.Draw(viewMatrix, projMatrix);
	}
	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	case GLFW_KEY_1:
		printf("Key 1 was %s\n", pressed ? "pressed" : "released");
		break; // don't forget this at the end of every "case" statement!
	default:
		break;
	}
}