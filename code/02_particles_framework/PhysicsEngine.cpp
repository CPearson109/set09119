#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;
using namespace std;

const glm::vec3 GRAVITY = glm::vec3(0, -10.0, 0);


void ExplicitEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Implement
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += impulse / mass;

	vel += accel * dt;

	pos += vel * dt;

	if (pos.y < 0.1f) {
		pos.y = 0.1f;
	}

}

vec3 CollisionImpulse(Particle& pobj, const vec3& cubeCentre, float cubeHalfExtent, float coefficientOfRestitution, vec3 impulse)
{

		vec3 v1 = pobj.Velocity();

		vec3 v2 = v1;

		v2.y = -v1.y * coefficientOfRestitution;

		impulse = pobj.Mass() * (v2 - v1);

	return impulse;
}

vec3 AerodynamicDrag(Particle& pobj, const vec3& velocity, float dragCoefficient, float airDensity, float crossSectionArea)
{

	float speed = length(pobj.Velocity());

	float dragMagnitude = 0.5f * airDensity * (speed * speed) * dragCoefficient * crossSectionArea;

	vec3 dragForce = -dragMagnitude * normalize(velocity);

	return dragForce;

}

vec3 CalculateWindForce(const Particle& pobj, const vec3& windForce, float dragCoefficient, float crossSectionArea)
{
	vec3 relativeWindVelocity = windForce - pobj.Velocity();

	float speed = length(relativeWindVelocity);

	float windMagnitude = 0.5f * (speed * speed) * dragCoefficient * crossSectionArea;

	vec3 windAcceleration = windMagnitude * normalize(relativeWindVelocity);

	return windAcceleration;
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

vec3 BlowDryerForce(const vec3& particlePosition, float cone_y_base, float cone_y_tip, float cone_r_base, float max_force = 100)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Calculate blow dryer force
	vec3 force = {0,0,0};
	return force;
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

	// Initialise particle

	particle.SetMesh(mesh);
	particle.SetShader(defaultShader);
	particle.SetColor(vec4(1, 0, 0, 1));
	particle.SetPosition(vec3(0, 5, 0));
	particle.SetScale(vec3(0.1f));
	particle.SetVelocity(vec3(1.f, 0.0f, 2.f));
	particle.SetMass(1.0f);

	camera = Camera(vec3(0, 2.5, 10));
}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
	// Adjust to alter energy loss on collision
	auto coefficientOfRestitution = 0.9f;

	// Adjust to alter aerodynamic drag
	const float airDensity = 1.225f;
	const float dragCoefficient = 0.5f;
	const float crossSectionArea = 1.0f;

	// Adjust to alter wind force
	vec3 windForce = vec3(1.0f, 0.0f, 0.5f);

	// Particle properties
	vec3 p = particle.Position();
	vec3 v = particle.Velocity();
	vec3 acceleration = vec3(GRAVITY);

	// Calculate aerodynamic drag force and its acceleration
	vec3 dragForce = AerodynamicDrag(particle, v, dragCoefficient, airDensity, crossSectionArea);
	vec3 dragAcceleration = dragForce / particle.Mass();

	// Calculate wind acceleration
	vec3 windAcceleration = CalculateWindForce(particle, windForce, dragCoefficient, crossSectionArea);

	// Initialize friction force
	vec3 frictionForce(0.0f);
	
	// Initialize impulse
	vec3 impulse(0.0f);

	if (particle.Position().y <= 0.1f) // Check if the particle is in contact with the ground
	{
		const float coefficientOfFriction = 0.5f; // Coefficient of friction for the surface
		vec3 normalForce = vec3(0, particle.Mass() * GRAVITY.y, 0); // Normal force equals mass * gravity for a horizontal surface
		frictionForce = CalculateFrictionForce(particle, coefficientOfFriction, normalForce);

		// Calculate collision impulse
		impulse = CollisionImpulse(particle, glm::vec3(0.0f, 5.0f, 0.0f), 5.0f, coefficientOfRestitution, impulse);
	}

	// Calculate total acceleration including drag, wind, and friction
	vec3 totalAcceleration = acceleration + dragAcceleration + windAcceleration + frictionForce / particle.Mass();

	// Update particle state using Symplectic Euler or any other integration method
	SymplecticEuler(p, v, particle.Mass(), totalAcceleration, impulse, deltaTime);
	particle.SetPosition(p);
	particle.SetVelocity(v);
}


// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	particle.Draw(viewMatrix, projMatrix);
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