#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;
using namespace std;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{

	vel += impulse / mass;

	vel += accel * dt;

	pos += vel * dt;

}

vec3 CollisionImpulse(Particle& pobj, const std::vector<RigidBody>& particles, float groundHeight, float wallXPos, float wallZPos, float coefficientOfRestitution, vec3 impulse)
{
	vec3 pos = pobj.Position();
	vec3 vel = pobj.Velocity();

	// Ground collision
	if (pos.y <= groundHeight && vel.y < 0) {
		vec3 groundNormal = vec3(0.0f, 1.0f, 0.0f);
		impulse -= pobj.Mass() * dot(vel, groundNormal) * groundNormal * (1 + coefficientOfRestitution);
		pos.y = groundHeight;
		float posx = pobj.Position().x;
		float posz = pobj.Position().z;
		pobj.SetPosition(vec3(posx, pobj.Scale().y, posz));

	}
	
	// Wall collisions - X-axis walls
	if ((pos.x <= -wallXPos && vel.x < 0) || (pos.x >= wallXPos && vel.x > 0)) {
		vec3 wallXNormal = vec3(pos.x < 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
		impulse -= pobj.Mass() * dot(vel, wallXNormal) * wallXNormal * (1 + coefficientOfRestitution);
	}
	
	// Wall collisions - Z-axis walls
	if ((pos.z <= -wallZPos && vel.z < 0) || (pos.z >= wallZPos && vel.z > 0)) {
		vec3 wallZNormal = vec3(0.0f, 0.0f, pos.z < 0 ? 1.0f : -1.0f);
		impulse -= pobj.Mass() * dot(vel, wallZNormal) * wallZNormal * (1 + coefficientOfRestitution);
	}

	////Particle collisions
	//for (const Particle& other : particles) {
	//	if (&pobj != &other) {
	//		vec3 otherPos = other.Position();
	//		vec3 deltaPos = pos - otherPos;
	//		float distance = length(deltaPos);
	//		float particleRadius = 0.1f;
	//
	//		// Check for collision
	//		if (distance < 2 * particleRadius) {
	//			vec3 collisionNormal = normalize(deltaPos);
	//			vec3 relativeVelocity = vel - other.Velocity();
	//
	//			float j = -(1 + coefficientOfRestitution) * dot(relativeVelocity, collisionNormal) / (1 / pobj.Mass() + 1 / other.Mass());
	//			impulse += j * collisionNormal;
	//		}
	//	}
	//}

	pobj.SetPosition(pos);

	return impulse;
}

vec3 CalculateFrictionForce(const Particle& pobj, float coefficientOfFriction, const vec3& normalForce)
{
	// The friction force is in the opposite direction of the particle's velocity on the surface
	vec3 frictionForce = -coefficientOfFriction * length(normalForce) * normalize(vec3(pobj.Velocity().x, 0.0f, pobj.Velocity().z));

	// Ensure friction only applies if the particle is actually moving on the surface
	if (length(pobj.Velocity()) > 0.0f)
		return frictionForce;
	else
		return vec3(0.0f);
}

void UpdateRigidBodyRotation(RigidBody& rb, float dt) {
	auto angVel = rb.AngularVelocity();
	mat3 angVelSkew = matrixCross3(angVel);
	mat3 R = mat3(rb.Orientation());
	R += dt * angVelSkew * R;
	R = orthonormalize(R);
	rb.SetOrientation(mat4(R));
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
		particle.SetVelocity(vec3(0.0f, 1.0f, 0.0f));
		particle.SetAngularVelocity(vec3(10.0f, 0.0f, 0.0f));
		particle.SetMass(1.0f);
		particles.push_back(particle);
	}

	camera = Camera(vec3(0, 2.5, 10));

}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{


	// Adjust to alter energy loss on collision
	auto coefficientOfRestitution = 0.9f;

	// Adjust to alter aerodynamic drag
	const float airDensity = 1.125f;
	const float dragCoefficient = 0.0f;
	const float crossSectionArea = 1.0f;

	// Adjust to alter wind force
	vec3 windForce = vec3(1.0f, 0.0f, 0.5f);

	int index = 0;

	// Particle properties
	for (auto& particle : particles) {

		vec3 p = particle.Position();
		vec3 v = particle.Velocity();
		vec3 acceleration = vec3(GRAVITY);

		// Initialize friction force
		vec3 frictionForce(0.0f);

		// Initialize impulse
		vec3 impulse(0.0f);


		if (particle.Position().y <= 0.1f) // Check if the particle is in contact with the ground
		{

			const float coefficientOfFriction = 0.5f; // Coefficient of friction for the surface
			vec3 normalForce = vec3(0, particle.Mass() * GRAVITY.y, 0); // Normal force equals mass * gravity for a horizontal surface
			frictionForce = CalculateFrictionForce(particle, coefficientOfFriction, normalForce);
		}

		// Calculate collision impulse
		impulse = CollisionImpulse(particle, particles, particle.Scale().y, 5.0f, 5.0f, coefficientOfRestitution, impulse);

		// Calculate total acceleration including drag, wind, and friction
		vec3 totalAcceleration = acceleration + frictionForce / particle.Mass();

		SymplecticEuler(p, v, particle.Mass(), totalAcceleration, impulse, deltaTime);


		particle.SetPosition(p);
		particle.SetVelocity(v);

		index++;

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