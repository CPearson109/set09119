#include "PhysicsEngine.h"
#include "Force.h"
#include "PhysicsObject.h"

using namespace glm;
using namespace std;

void Force::Gravity(Particle& p)
{
	auto force = vec3(0, -9.81, 0) * p.Mass();
	p.ApplyForce(force);
}

void Force::Drag(Particle& p)
{
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
	// TODO: Should apply the aerodynamic drag force
	//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
}

void Force::Hooke(Particle& p1, Particle& p2, float restLength, float springConstant, float dampingCoefficient) {

    vec3 p1_to_p2 = p2.Position() - p1.Position(); // Vector from p1 to p2
    float currentLength = length(p1_to_p2); // Current length of the spring

    // Normalize the vector between the partcles
    vec3 direction = normalize(p1_to_p2);

    // Calculate the spring force magnitude
    float springForceMagnitude = -springConstant * (currentLength - restLength);

    vec3 relativeVelocity = p2.Velocity() - p1.Velocity();
    float dampingForceMagnitude = -dampingCoefficient * dot(relativeVelocity, direction);


    vec3 totalForce = (springForceMagnitude + dampingForceMagnitude) * direction;

    // Apply the force to both particles in opposite directions
    p1.ApplyForce(-totalForce);
    p2.ApplyForce(totalForce);
}

void Force::Wind(Particle& p, float dt) {

    float randForce;

    randForce = (float) (rand()) / (float)(RAND_MAX);

    auto windForce = vec3(p.Position().x, p.Position().y, (randForce * dt) / 10);

    p.ApplyForce(windForce);
}
