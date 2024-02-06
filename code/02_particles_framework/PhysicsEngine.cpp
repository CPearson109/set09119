#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"

using namespace glm;
using namespace std;

const glm::vec3 GRAVITY = glm::vec3(0, -10.0, 0);


void ExplicitEuler(vec3& pos, vec3& prevPos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{


}

void Verlet(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += impulse / mass;

	pos = pos + vel * dt + 0.5f * accel * (dt * dt);

	vel = vel + 0.5f * (accel + accel) * dt;

}


void SymplecticEuler(vec3& pos, vec3& vel, float mass, const vec3& accel, const vec3& impulse, float dt)
{
	vel += impulse / mass;

	vel += accel * dt;

	pos += vel * dt;

}

vec3 CollisionImpulse(Particle& pobj, float groundHeight, float wallXPos, float wallZPos, float coefficientOfRestitution, vec3 impulse)
{
	const vec3 pos = pobj.Position();
	const vec3 v1 = pobj.Velocity();

	// Ground collision
	if (pos.y <= groundHeight) {
		vec3 groundNormal = vec3(0.0f, 1.0f, 0.0f);
		impulse -= (pobj.Mass() * dot(v1, groundNormal) * groundNormal) * coefficientOfRestitution;
	}

	// Wall collisions - X-axis walls
	if (pos.x <= -wallXPos || pos.x >= wallXPos) {
		vec3 wallXNormal = vec3(pos.x < 0 ? 1.0f : -1.0f, 0.0f, 0.0f);
		impulse -= (pobj.Mass() * dot(v1, wallXNormal) * wallXNormal) * coefficientOfRestitution;
	}
	
	// Wall collisions - Z-axis walls
	if (pos.z <= -wallZPos || pos.z >= wallZPos) {
		vec3 posz = pobj.Position();
		posz.z = posz.z > wallZPos ? posz.z = wallZPos : posz.z = -wallZPos;
		pobj.SetPosition(posz);
		vec3 wallZNormal = vec3(0.0f, 0.0f, pos.z < 0 ? 1.0f : -1.0f);
		impulse -= (pobj.Mass() * dot(v1, wallZNormal) * wallZNormal) * coefficientOfRestitution;
	}

	pobj.SetPosition(pos);
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

	// Initialise particle comlour and position arrays
	vec4 particleColour[2];
	vec3 particlePosition[2];

	// Initialize elements
	particleColour[0] = vec4(1.0, 0.0, 0.0, 1.0); // red
	particleColour[1] = vec4(0.0, 1.0, 0.0, 1.0); // green

	particlePosition[0] = vec3(-1.0f, 5.0f, 0.0f);
	particlePosition[1] = vec3(1.0f, 5.0f, 0.0f);

	for (int i = 0; i < 2; ++i) {
		Particle particle;
		particle.SetMesh(mesh);
		particle.SetShader(defaultShader);
		particle.SetColor(vec4(particleColour[i]));
		particle.SetPosition(vec3(particlePosition[i]));
		particle.SetPrevPos(vec3(0.0f, 0.0f, 0.0f));
		particle.SetScale(vec3(0.1f));
		particle.SetVelocity(vec3(2.0f, 0.0f, 50.0f));
		particle.SetMass(1.0f);
		particles.push_back(particle);
	}

	camera = Camera(vec3(0, 2.5, 10));

}

// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{


	// Adjust to alter energy loss on collision
	auto coefficientOfRestitution = 1.9f;

	// Adjust to alter aerodynamic drag
	const float airDensity = 1.125f;
	const float dragCoefficient = 0.0f;
	const float crossSectionArea = 1.0f;

	// Adjust to alter wind force
	vec3 windForce = vec3(1.0f, 0.0f, 0.5f);

	int index = 0;

	// Particle properties
	for (auto& particle : particles) {

		if (particle.Position().y < 0.1f) {
			float posx = particle.Position().x;
			float posz = particle.Position().z;

			particle.SetPosition(vec3(posx, 0.1f, posz));
		}

		vec3 prevPos = particle.PrevPos();
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
		}

		// Calculate collision impulse
		impulse = CollisionImpulse(particle, 0.1f, 5.0f, 5.0f, coefficientOfRestitution, impulse);

		// Calculate total acceleration including drag, wind, and friction
		vec3 totalAcceleration = acceleration + dragAcceleration + windAcceleration + frictionForce / particle.Mass();


		if (index == 0) {
			// Update particle state using Symplectic Euler or any other integration method
			SymplecticEuler(p, v, particle.Mass(), totalAcceleration, impulse, deltaTime);

			//cout << "Position CUBE 1: (" << p.x << ", " << p.y << ", " << p.z << ")" << endl;
		}
		else if (index == 1) {
			Verlet(p, v, particle.Mass(), totalAcceleration, impulse, deltaTime);

			cout << impulse.y << endl;

			//cout << "Position CUBE 2: (" << p.x << ", " << p.y << ", " << p.z << ")" << endl;


		}

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