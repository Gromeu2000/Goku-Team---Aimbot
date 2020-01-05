#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <math.h>
#include "EulerIntegratorAndAimBot.h"

using namespace std;
#define PI 3.1416f
#define GRAVITY -9.8f
#define ORIGIN vec3d({0.0f, 0.0f, 0.0f})
#define ITERATIONS 1000
#define MAX_ANGLE 180.0f
#define MAX_WIND_VEL 10.0f
#define POS_MAX_RAND 10

int main()
{
	// --- Monte-Carlo
	InitSimulation();
	Monte_Carlo(ITERATIONS, projectile, target);

	system("pause");
	return 0;
}

// ------------------------------------------ AIMBOT / MONTE-CARLO ------------------------------------------
void AimBotEulerIntegrator(Particle& projectile, Particle& target)
{
	cout << "Fluid Velocity: (" << world.fluidVelocity.x << " " << world.fluidVelocity.y << " " << world.fluidVelocity.z << ")" << endl;

	for (int i = 0; i < world.total_time; i++)
	{
		world.fg = projectile.mass * world.gravity;													//Calculates the gravitational force applied to the projectile.
		
		world.totalVel = projectile.speed - world.fluidVelocity;									//Calculates the total velocity vector from the projectile's speed vector and the fluid velocity vector.

		TotalVelSafetyCheck(world.totalVel);														//Checks that world.totalVel vector is not 0 (world.totalVel.norm() == 0).
		
		world.uVel = world.totalVel / world.totalVel.norm();										//Calculates the unitary particle-wind velocity vector. Will always be opposed to the projectile's velocity vector.

		world.fd = 0.5f * world.fluidDensity * world.totalVel.x * world.totalVel.x * projectile.dragCoefficient * projectile.surface * -world.uVel.x;	//Calculates the drag force applied to the projectile.

		projectile.acceleration.x = world.fd / projectile.mass;										//Calculates the acceleration of the projectile for the X axis.
		projectile.acceleration.y = world.fg / projectile.mass;										//Calculates the acceleration of the projectile for the Y axis.
		projectile.acceleration.z = projectile.acceleration.z;										//Calculates the acceleration of the projectile for the Z axis.

		//y = yo + vo * dt
		//v = vo + a * dt
		projectile.position.x = projectile.position.x + projectile.speed.x * world.dt;				//Gets the object's final position in the X axis.
		projectile.position.y = projectile.position.y + projectile.speed.y * world.dt;				//Gets the object's final position in the Y axis.
		projectile.position.z = projectile.position.z + projectile.speed.z * world.dt;				//Gets the object's final position in the Z axis.

		projectile.speed.x = projectile.speed.x + projectile.acceleration.x * world.dt;				//Gets the object's final velocity in the X axis.
		projectile.speed.y = projectile.speed.y + projectile.acceleration.y * world.dt;				//Gets the object's final velocity in the Y axis.
		projectile.speed.z = projectile.speed.z + projectile.acceleration.z * world.dt;				//Gets the object's final velocity in the Z axis.
		
		CheckRebound(projectile);																	//Checks whehter or not the projectile has collided against a wall. 
																									//If there has been a collision the velocity vector will be flipped/inverted.

		if (CheckHit(projectile, target))															//Checks whether or not the projectile has collided against the target.
		{
			aimbot.targetWasHit = true;																//If the target was hit, the targetWasHit flag will be set to true and the loop will be terminated.
			break;
		}
	}
}

void Monte_Carlo(int iterations, Particle& projectile, Particle& target)
{
	for (int i = 0; i < iterations; i++)
	{
		cout << "Monte-Carlo " << i << endl;
		
		projectile.position			= ORIGIN;														// Resetting the projectile's position back to ORIGIN.

		RandomizeVelocityAndAngle();																// Method that randomizes both the initial velocity module and the throwing angle.

		PropagateAll(projectile, target, aimbot.velModule, aimbot.angle);							// Running the integrator to propagate the state of the projectile.

		if (aimbot.targetWasHit)																	// If the target was hit...
		{	
			cout << endl;

			cout << "Target at (" << target.position.x << " " << target.position.y << " " << target.position.z <<
				") was hit at iteration " << i << " of the Monte-Carlo method." << endl;

			cout << "Final Speed: (" << projectile.speed.x << " " << projectile.speed.y << " " << projectile.speed.z << ")" << endl;
			cout << "Throwing Angle: " << aimbot.angle << endl;

			break;
		}
	}
}

void PropagateAll(Particle& projectile, Particle& target, float velModule, float angle)
{
	projectile.speed.x = velModule * cos(angle);
	projectile.speed.y = velModule * sin(angle);
	projectile.speed.z = 0.0f;

	cout << "Initial position: (" << projectile.position.x << " " << projectile.position.y << " " << projectile.position.z << ")" << endl;
	cout << "Initial velocity: (" << projectile.speed.x << " " << projectile.speed.y << " " << projectile.speed.z << ")" << endl;
	cout << "Initial acceleration: (" << projectile.acceleration.x << " " << projectile.acceleration.y << " " << projectile.acceleration.z << ")" << endl;
	cout << "Initial angle: " << angle << endl;
	cout << "Target position: (" << target.position.x << " " << target.position.y << " " << target.position.z << ")" << endl;
	
	AimBotEulerIntegrator(projectile, target);

	cout << "fpos is: (" << projectile.position.x << " " << projectile.position.y << " " << projectile.position.z << ")";
	cout << "	fvel is: (" << projectile.speed.x << " " << projectile.speed.y << " " << projectile.speed.z << ")" << endl;

	if (CheckHit(projectile, target))														// This check here makes the projectile be like a grenade, as the check 
	{																						// happens after the projectile has stopped being propagated.
		aimbot.targetWasHit = true;
	}

	cout << "Final position: (" << projectile.position.x << " " << projectile.position.y << " " << projectile.position.z << ")" << endl;

	cout << endl;
}

// ------------------------------------- SIMULATION CHECKS --------------------------------------
bool CheckHit(const Particle& projectile, const Particle& target)							// ----------------------------------------------------------------------------------------------
{
	if (DistBetweenElements(projectile.position, target.position) <= target.radius)			// If the distance between the projectile and the target is less than the target's radius...
	{
		return true;																		// CheckHit() will return true as here has been a collision/hit between the projectile and the target.
	}

	return false;
}

float DistBetweenElements(vec3d projectilePos, vec3d targetPos)								// ----------------------------------------------------------------------------------------------
{
	vec3d distBuffer = projectilePos - targetPos;

	float distNoSqrt = distBuffer.norm();

	float dist = sqrt(distNoSqrt);

	return dist;
	//return distNoSqrt;
}

void CheckRebound(Particle& projectile)														// ----------------------------------------------------------------------------------------------
{
	// --- Projectile hits the walls on the X Axis.
	if (projectile.position.x <= 0.0f || projectile.position.x >= world.worldWidth)			// If the projectile's position in the X axis is less than 0.0f or more than the world's width...
	{
		projectile.speed.x = -(projectile.restitutionCoefficient * projectile.speed.x);		// The component x of the vector speed will be flipped/inverted.
	}

	// --- Projectile hits the walls on the Y Axis.
	if (projectile.position.y <= 0.0f || projectile.position.y >= world.worldHeight)		// If the projectile's position in the Y axis is less than 0.0f or more than the world's height...
	{
		projectile.speed.y = -(projectile.restitutionCoefficient * projectile.speed.y);		// The component y of the vector speed will be flipped/inverted.
	}
}

void TotalVelSafetyCheck(vec3d& totalVel)													// ----------------------------------------------------------------------------------------------
{
	if (totalVel.norm() < world.minVel)														// If the norm of totalVel's vector is less than the minimum velocity threshold of the world...
	{
		totalVel = vec3d(world.minVel, world.minVel, 0.0f);									// totalVel vector components are set to the world's minimum velocity threshold.
	}
}
// ----------------------------------------------------------------------------------------------

// ---------------------------- INITIALIZING ALL SIMULATION ELEMENTS ----------------------------
void InitSimulation()																	// ----------------------------------------------------------------------------------------------
{
	InitSimulationWorld();																// Initializes the simulation world's variables.
	InitSimulationElements();															// Initializes the simulation's elements (projectile and target).

	RandomizeWindVelocity();															// Sets the simulation world's wind velocity with a random value.
}

void InitSimulationWorld()																// ----------------------------------------------------------------------------------------------
{
	world.gravity			= GRAVITY;													// Gravity will be -9.8f.
	world.worldWidth		= 20;														// World Width will be 20m.
	world.worldHeight		= 10;														// World Height will be 10m.
	world.fluidVelocity		= ORIGIN;													// Fluid velocity will be a random value (-10.0f ~ 10.0f)
	world.fluidDensity		= 1.2f;														// Fluid Density will be 1.2f (Air, 20º at 1 atm).
	world.simulation_fps	= 60.0f;													// The simulation will run at 60 FPS. Change later to match real time (syncronized with the game).	
	world.dt				= 1.0f / world.simulation_fps;								// Dt will be 0.016f (1 / 60).	
	world.simulation_time	= 5.0f;														// For each simulation case, the simulation will run for 5 seconds. (5s of propagation).
	world.total_time		= world.simulation_fps * world.simulation_time;				// Total amount of frames that the AimBot will propagate the state of the projectile for.
	world.minVel			= 0.0001f;													// The minimum velocity threshold will be 0.0001 m/s.
}

void InitSimulationElements()															// ----------------------------------------------------------------------------------------------
{
	// --- INITIALIZING THE PROJECTILE

	projectile.position					= ORIGIN;										// Initial Position will be set at ORIGIN ({0.0f, 0.0f, 0.0f})
	projectile.speed					= ORIGIN;										// Velocity will be randomized (0.0f ~ 50.0f) at the MonteCarlo() method.
	projectile.acceleration				= ORIGIN;										// The Acceleration applied to the projectile will be the World's gravity.
	projectile.mass						= 1.0f;											// Mass will be set to 1 Kg.
	projectile.radius					= 0.5f;											// Radius will be set to 0.5m.
	projectile.surface					= PI * projectile.radius * projectile.radius;	// As the projectile will have a radius of 0.5m, the Surface Area will be 0.78 m^2.
	projectile.dragCoefficient			= 0.47f;										// The Drag Coefficient (Cd) of a sphere is 0.47.
	projectile.restitutionCoefficient	= 0.95f;										// The Restitution Coefficient of the projectile will be set to 0.95 (0 ~ 1)
	
	// --- INITIALIZING THE TARGET
	vec3d targetPos				= { (float)(std::rand() % POS_MAX_RAND), (float)(std::rand() % POS_MAX_RAND), 0.0f };
	vec3d targetSpeed			= ORIGIN;
	vec3d targetAcceleration	= ORIGIN;

	// Initial Target Position will be randomized (for now).
	// Target Radius will be set to 1.0m.
	// As the target will be static:
	//	- Speed will be NIL.
	//	- Acceration will be NIL.
	//	- All other variables will be set to 0.0f (default).
	
	target = Particle(targetPos, targetSpeed, targetAcceleration);

	aimbot.targetWasHit = false;														//If this bool will be set to true when the projectile hits the target. Stops the Monte-Carlo.
}
// ----------------------------------------------------------------------------------------------

// ---------------------------- RANDOMIZING MONTE-CARLO'S VARIABLES -----------------------------
void RandomizeVelocityAndAngle()														// ----------------------------------------------------------------------------------------------
{
	aimbot.velModule	= (float)(std::rand() % 50);									// The projectile's velocity module is set with a random value. (0.0 m/s ~ 50.0 m/s)
	aimbot.angle		= -MAX_ANGLE + (float)(std::rand() % 360);						// The projectile's throwing angle is set with a random value.  (-180.0º ~ 180.0º)
}

void RandomizeWindVelocity()															// ----------------------------------------------------------------------------------------------
{
	world.fluidVelocity = { -MAX_WIND_VEL + (float)(std::rand() % 20), 0.0f, 0.0f };	//	The world's fluidVelocity (wind velocity) is set with a random value. (-10.0 m/s ~ 10.0 m/s)
}
// ----------------------------------------------------------------------------------------------

// -------------------------------- CLASS CONSTRUCTORS & METHODS --------------------------------
// --- VEC3D CLASS
vec3d::vec3d()
{

}

vec3d::vec3d(float x, float y, float z)
{
	this->x = x;
	this->y = y;
	this->z = z;
}

float vec3d::norm()
{
	float normV;

	float bufferNorm = this->x * this->x + this->y * this->y + this->z * this->z;

	normV = sqrt(bufferNorm);

	return normV;
}

const vec3d& vec3d::operator +(const vec3d &vec)
{
	vec3d r;

	r.x = x + vec.x;
	r.y = y + vec.y;
	r.z = z + vec.z;

	return(r);
}

const vec3d& vec3d::operator -(const vec3d &vec)
{
	vec3d r;

	r.x = x - vec.x;
	r.y = y - vec.y;
	r.z = z - vec.z;

	return(r);
}

const vec3d& vec3d::operator /(const vec3d &vec)
{
	x = x / vec.x;
	y = y / vec.y;
	z = z / vec.z;

	return (*this);
}

const vec3d& vec3d::operator /(const float &v)
{
	vec3d r;

	r.x = x / v;
	r.y = y / v;
	r.z = z / v;

	return(r);
}
//-------------------

// --- PARTICLE CLASS
Particle::Particle()
{

}

Particle::Particle(vec3d position, vec3d speed, vec3d acceleration, float mass, float radius, float surface, float dragCoefficient, float restitutionCoefficient)
{
	this->position					= position;
	this->speed						= speed;
	this->acceleration				= acceleration;
	this->mass						= mass;
	this->radius					= radius;
	this->surface					= surface;
	this->dragCoefficient			= dragCoefficient;
	this->restitutionCoefficient	= restitutionCoefficient;
}
//-------------------

// --- AIMBOTVARIABLES CLASS
AimBotVariables::AimBotVariables()
{

}

AimBotVariables::AimBotVariables(float velModule, float angle, bool targetWasHit)
{
	this->velModule					= velModule;
	this->angle						= angle;
	this->targetWasHit				= targetWasHit;
}
//-------------------

// --- WORLD CLASS
World::World()
{

}

World::World(float gravity, int worldWidth, int worldHeight, vec3d fluidVelocity, float fluidDensity, float simulation_fps, float simulation_time)
{
	this->gravity					= gravity;
	this->worldWidth				= worldWidth;
	this->worldHeight				= worldHeight;
	this->fluidVelocity				= fluidVelocity;
	this->fluidDensity				= fluidDensity;

	this->simulation_fps			= simulation_fps;
	this->dt						= 1.0f / simulation_fps;
	this->simulation_time			= simulation_time;
	this->total_time				= simulation_fps * simulation_time;
}
//-------------------
// ----------------------------------------------------------------------------------------------