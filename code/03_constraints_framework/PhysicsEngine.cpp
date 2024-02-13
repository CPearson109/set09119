#include "PhysicsEngine.h"
#include "Application.h"
#include "Camera.h"
#include "Force.h"

using namespace glm;
using namespace std;

const glm::vec3 GRAVITY = glm::vec3(0, -9.81, 0);

const int gridWidth = 3;
const int gridHeight = 3;

void SymplecticEuler(Particle& pobj, vec3 totalForce, vec3 totalImpulse, float mass, float dt)
{
	vec3 vel = pobj.Velocity();
	vec3 pos = pobj.Position();
	

	vel += totalImpulse / mass;

	vel += totalForce * dt;

	pos += vel * dt;

	pobj.SetPosition(pos);
	pobj.SetVelocity(vel);

}

vec3 CollisionImpulse(Particle& pobj, const std::vector<Particle>& particles, float groundHeight, float wallXPos, float wallZPos, float coefficientOfRestitution, vec3 impulse)
{
	vec3 pos = pobj.Position();
	vec3 vel = pobj.Velocity();

	// Ground collision
	if (pos.y <= groundHeight && vel.y < 0) {
		vec3 groundNormal = vec3(0.0f, 1.0f, 0.0f);
		impulse -= pobj.Mass() * dot(vel, groundNormal) * groundNormal * (1 + coefficientOfRestitution);
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

	return impulse;
}

// This is called once
void PhysicsEngine::Init(Camera& camera, MeshDb& meshDb, ShaderDb& shaderDb) {
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

	for (int y = 0; y < gridHeight; y++) {
		for (int x = 0; x < gridWidth; x++) {
			Particle particle;
			particle.SetMesh(mesh);
			particle.SetShader(defaultShader);
			particle.SetColor(vec4(1.0f, 0.0f, 0.0f, 1.0f));
			particle.SetPosition(vec3(0.0f + x + y, 10.0f - y, 0.0f));
			particle.SetScale(vec3(0.1f));
			particle.SetVelocity(vec3(0.0f, 0.0f, 0.0f));
			particle.SetMass(1.0f);
			particles.push_back(particle);
		}
	}

	camera = Camera(vec3(0, 2.5, 10));
}


void PhysicsEngine::Task1Init()
{

}

void PhysicsEngine::Task1Update(float deltaTime, float totalTime) {

	float coefficientOfRestitution = 0.7f;

	int index = 0;

	float dampingCoefficient = 0.1f;
	float springConstant = 10.0f;
	float restLength = 1.0f;

	for (Particle& particle : particles) {

		if (index > gridWidth - 1){

			vec3 impulse(0.0f);

			impulse = CollisionImpulse(particle, particles, 0.1f, 5.0f, 5.0f, coefficientOfRestitution, impulse);

			particle.ApplyImpulse(impulse);

			Force::Gravity(particle);

			for (int y = 0; y < gridHeight; y++) {
				for (int x = 0; x < gridWidth; x++) {

					int index = y * gridWidth + x;
					Particle& particle = particles[index];

					// above
					if (y > 0) {
						Force::Hooke(particle, particles[(y - 1) * gridWidth + x], restLength, springConstant, dampingCoefficient);
						//cout << index << " - " << (y - 1) * gridWidth + x << endl;
					}

					// below
					if (y < gridHeight - 1) {
						Force::Hooke(particle, particles[(y + 1) * gridWidth + x], restLength, springConstant, dampingCoefficient);
						//cout << index << " - " << (y + 1) * gridWidth + x << endl;
					}

					// left
					if (x > 0) {
						Force::Hooke(particle, particles[y * gridWidth + (x - 1)], restLength, springConstant, dampingCoefficient);
						//cout << index << " - " << y * gridWidth + (x - 1) << endl;
					}

					// right
					if (x < gridWidth - 1) {
						Force::Hooke(particle, particles[y * gridWidth + (x + 1)], restLength, springConstant, dampingCoefficient);
						//cout << index << " - " << y * gridWidth + (x + 1) << endl;
					}

					// top left
					if (x > 0 && y > 0) {
						Force::Hooke(particle, particles[(y - 1) * gridWidth + (x - 1)], restLength, springConstant, dampingCoefficient);
						cout << index << " - " << y - 1 * gridWidth + (x + 1) << endl;
					}
				

					//top right
					if (x > gridWidth - 1 && y > 0) {

					}

					//bottom left
					if (y < gridHeight - 1 && x > 0) {

					}

					//bottom right
					if (y < gridHeight - 1 && y > 0) {

					}
				}
			}


		}

		index++;
	}

	index = 0;

	for (Particle& particle : particles) {

		float mass = particle.Mass();

		vec3 totalForce = particle.AccumulatedForce();

		vec3 totalImpulse = particle.AccumulatedImpulse();

		if (index > gridWidth - 1) {
			SymplecticEuler(particle, totalForce, totalImpulse, mass, deltaTime);
		}

		particle.ClearForcesImpulses();

		index++;
	}

}


// This is called every frame
void PhysicsEngine::Update(float deltaTime, float totalTime)
{
		Task1Update(deltaTime, totalTime);
}

// This is called every frame, after Update
void PhysicsEngine::Display(const mat4& viewMatrix, const mat4& projMatrix)
{
	for (Particle& particle : particles) {
		particle.Draw(viewMatrix, projMatrix);
	}

	ground.Draw(viewMatrix, projMatrix);
}

void PhysicsEngine::HandleInputKey(int keyCode, bool pressed)
{
	switch (keyCode)
	{
	default:
		break;
	}
}