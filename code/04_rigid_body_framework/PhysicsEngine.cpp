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

	if (contactPoints.empty()) return;


	for (const auto& point : contactPoints) {
		// Calculate penetration depth for each contact point
		float penetrationDepth = groundHeight - point.y;

		// Apply correction for each contact point
		vec3 correction = vec3(0.0f, penetrationDepth, 0.0f);
		rigidBody.SetPosition(rigidBody.Position() + correction);

	}

}

void Impulse(RigidBody& rigidBody, const std::vector<vec3>& contactPoints) {
	const float velThres = 0.1f;
	float coefficientOfRestitution = 0.7f;
	float frictionCoefficient = 0.0f;

	for (const auto& point : contactPoints) {
		vec3 r = point - rigidBody.Position(); // Vector from COM to contact point
		vec3 n = vec3(0, 1, 0); // Collision normal
		vec3 V_rel = rigidBody.Velocity() + cross(rigidBody.AngularVelocity(), r); // Relative velocity at contact point

		// Normal impulse calculation
		float numerator = -(1 + coefficientOfRestitution) * dot(V_rel, n);
		float term1 = 1 / rigidBody.Mass();
		float term2 = dot(cross(r, n), cross(r, n) * rigidBody.InverseInertia());
		float J = numerator / (term1 + term2); // Impulse magnitude

		// Friction impulse calculation
		vec3 tangent = normalize(V_rel - dot(V_rel, n) * n); // Tangential direction
		float J_friction = dot(V_rel, tangent) / (term1 + term2); // Magnitude of friction impulse
		J_friction = std::min(J_friction, J * frictionCoefficient); // Limit friction impulse

		vec3 frictionImpulse = -J_friction * tangent;

		// Apply impulses
		rigidBody.SetVelocity(rigidBody.Velocity() + (J * n + frictionImpulse) / rigidBody.Mass());
		rigidBody.SetAngularVelocity(rigidBody.AngularVelocity() + rigidBody.InverseInertia() * cross(r, J * n + frictionImpulse));

		if (length(rigidBody.Velocity()) < velThres) {
			rigidBody.SetVelocity(vec3(0.0f));
		}
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

vec3 ApplyImpulseAtPoint(RigidBody& rb, const vec3& linImpulse, const vec3& angImpulse, const vec3& point) {

	// Update linear velocity
	vec3 deltaV = linImpulse;
	vec3 vel = rb.Velocity();
	vel += deltaV;

	vec3 r = point - rb.Position();

	// Update angular velocity
	glm::mat3 invInertiaWorld = rb.InverseInertia();
	vec3 deltaOmega = invInertiaWorld * glm::cross(r, angImpulse);
	vec3 angVel = rb.AngularVelocity();
	angVel += deltaOmega;

	rb.SetAngularVelocity(angVel);


	return vel;

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
		particle.SetPosition(vec3(0.0f, 10.0f, 0.0f));
		particle.SetScale(vec3(1.0f, 3.0f, 1.0f));
		particle.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
		particle.SetAngularVelocity(vec3(100.0f, 0.0f, 0.0f));
		particle.SetMass(1.0f);
		particles.push_back(particle);
	}

	camera = Camera(vec3(0, 2.5, 10));

}

int testNumber = 0;
bool applyImpulse = false;
bool impulseApplied = false;
float accumulatedTime = 0.0f;
vec3 linImpulse = vec3(0.0f, 0.0f, 0.0f);
vec3 angImpulse = vec3(0.0f, 0.0f, 0.0f);
vec3 impulsePoint = vec3(0.0f, 0.0f, 0.0f);

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	accumulatedTime += deltaTime;

	if (accumulatedTime >= 2.0f && impulseApplied == false) {
		applyImpulse = true;
		impulseApplied = true;
	}

	int groundHeight = 0;
	vector<vec3> contactPoints;

	for (auto& particle : particles) {
		vec3 p = particle.Position();
		vec3 v = particle.Velocity();
		vec3 acceleration = GRAVITY;
		vec3 impulse(0.0f);

		if (applyImpulse == true) {
			if (testNumber == 1 && accumulatedTime < 2.0f) {


			}
			else if (testNumber == 1 && accumulatedTime > 2.0f) {

				linImpulse = vec3(-4.0f, 0.0f, 0.0f);
				angImpulse = vec3(0.0f, 0.0f, 0.0f);
			}
			if (testNumber == 2 && accumulatedTime < 2.0f) {

			}
			else if (testNumber == 2 && accumulatedTime > 2.0f) {

				linImpulse = vec3(-1.0f, 0.0f, 0.0f);
				angImpulse = vec3(0.0f, 0.0f, -1.0f);
			}

			vec3 updatedVelocity = ApplyImpulseAtPoint(particle, linImpulse, angImpulse, impulsePoint);
			// Directly use the updated velocity for the next Symplectic Euler update
			impulse = updatedVelocity;

			applyImpulse = false;
		}

		// handle the collision detection
		contactPoints.clear(); // Clear previous contact points
		CheckForCollision(particle, groundHeight, contactPoints);

		// If contact points, handle the collision response
		if (!contactPoints.empty()) {

			Impulse(particle, contactPoints);
			ResolveCollision(particle, contactPoints, groundHeight);

			p = particle.Position();
			v = particle.Velocity();
		}
			// If no collision, regular update
			vec3 totalAcceleration = acceleration;

			// Update position and velocity using Symplectic Euler
			SymplecticEuler(p, v, particle.Mass(), totalAcceleration, impulse, deltaTime);

		// Update the rigid bodys rotation
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

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed) {
	if (!pressed) return;

	switch (keyCode) {
	case GLFW_KEY_1:
		testNumber = 1;
		break;
	case GLFW_KEY_2:
		testNumber = 2;
		break;
	default:
		return;
	}

	for (auto& particle : particles) {
		particle.SetPosition(vec3(0.0f, 10.0f, 0.0f)); 
		if (testNumber == 1) {
			particle.SetVelocity(vec3(2.0f, 0.0f, 0.0f));
		}
		else {
			particle.SetVelocity(vec3(0.0f));
		}
		particle.SetAngularVelocity(vec3(0.0f));
		particle.SetOrientation(mat4(1.0f));
	}

	// Reset control variables
	accumulatedTime = 0.f;
	impulseApplied = false;
	applyImpulse = false;
}
