

class vec3d				//Class that will be used to declare 3D Vectors.
{
public:
	vec3d();
	vec3d(float x, float y, float z);
	
	float norm();

	const vec3d& operator +(const vec3d &vec);
	const vec3d& operator -(const vec3d &vec);
	const vec3d& operator /(const vec3d &vec);
	const vec3d& operator /(const float &v);

public:
	float	x;
	float	y;
	float	z;
};

class Particle			//Class that will be used to declare objects / particles. All particles will be spherical.
{
public:
	Particle();
	Particle(vec3d position, vec3d speed, vec3d acceleration, float mass = 0.0f, float radius = 1.0f, float surface = 0.0f, float dragCoefficient = 0.0f, float restitutionCoefficient = 0.0f);
	
public:
	vec3d	position;								//Position of the particle in space.
	vec3d	speed;									//Speed of the particle.
	vec3d	acceleration;							//Acceleration of the particle.
	float	mass;									//Mass of the particle.
	float	radius;									//Radius of the particle.
	float	surface;								//Front surface area of a particle.
	float	dragCoefficient;						//Drag coefficient of the particle.
	float	restitutionCoefficient;					//Restitution coefficient of the particle.
};

class AimBotVariables	//Class that will be used to declare miscelaneous variables related with the AimBot and the intercting variables of the simulation world (fg, fd, totalVel...).
{
public:
	AimBotVariables();
	AimBotVariables(float velModule, float angle, bool targetWasHit);

	float	velModule;								//Velocity module that will be randomized and then applied to the projectile's speed.
	float	angle;									//Angle at which the projectile is trown.
	bool	targetWasHit;							//Flag that keeps track of whether or not the projectile has hit the target.
};

class World				//Class that will be used to define the simulation world's base properties.
{
public:
	World();
	World(float gravity, int worldWidth, int worldHeight, vec3d fluidVelocity, float fluidDensity, float simulation_fps, float simulation_time);
	
public:
	float	gravity;								//Gravity of the simulation world.
	float	worldWidth;								//Width of the simulation world.
	float	worldHeight;							//Height of the simulation world.
	vec3d	fluidVelocity;							//Fluid velocity in the simulation world. (Air, Water...)
	float	fluidDensity;							//Fluid density in the simulation world. (Air, Water...)

	float	simulation_fps;							//Average fps of the simulation.
	float	dt;										//Timestep of the world (1/fps --> 1/60 = 0.016s)
	float	simulation_time;						//Total amount of time simulated for a specific Monte-Carlo case. In Seconds.
	float	total_time;								//Total time allocated to Monte-Carlo.

	float	f;										//Total amount of force that the projectile is subjected to by the Simulation World. [F = ma];
	float	fg;										//Gravitational Force of the Simulation World. [Fg = mg];
	float	fd;										//Drag Force of the Simulation World. [Fd = 0.5 * rho * v^2 * Cd * A];
	vec3d	totalVel;								//Velocity variable of Fd. Only takes into account the X Axis: [totalVel = projectile.speed.x - world.fluidVelocity];
	vec3d	uVel;									//Unitary particle-wind velocity vector. [uVel = totalVel / totalVel.norm()];
	float	minVel;									//Threshold value for the minimum velocity allowed in the world [m/s]
};

// --- AIMBOT
// -- Methods
void InitSimulation();																			//Method that calls both InitSimulationWorld() and InitSimulationElements(). Done for readability.
void InitSimulationWorld();																		//Initializes the variables of the Simulation World (gravity, boundaries...)
void InitSimulationElements();																	//Initializes the variables of the elements of the simulation (projectile and target)

void RandomizeVelocityAndAngle();																//Method that randomizes projectile.speed and angle.
void RandomizeWindVelocity();																	//Method that randomizes fluidVelocity.

void AimBotEulerIntegrator(Particle& projectile, Particle& target);								//Euler Integrator adapted to take into account the drag force. Takes a projectile and a target as arguments.
void Monte_Carlo(int iterations, Particle& projectile, Particle& target);						//Randomizes projectile.speed, angle and fluidVelocity and calls the PropagateAll() mehtod.Takes a number of iterations along with a projectile and a target as arguments.
void PropagateAll(Particle& projectile, Particle& target, float velModule, float angle);		//Propagates the state of the projectile. Calls the AimbotEulerIntegrator() method.

bool CheckHit(const Particle& projectile, const Particle& target);								//Checks whether the projectile has hit the target or not.
float DistBetweenElements(vec3d projectilePos, vec3d targetPos);								//Calculates the distance between the two positions passed as arguments.
void CheckRebound(Particle& projectile);														//Checks whether the projectile has collided against a wall or not.
void TotalVelSafetyCheck(vec3d& totalVel);														//Checks that the totalVel vector is not below the world.minVel threshold, which would break the aimbot as totalVel.norm() = 0;

// -- Variables
World				world;													//Simulation World where the projectile and the Target exist.
AimBotVariables		aimbot;													//AimBot relevant variables (angle, targetWasHit...)
Particle			projectile;												//Projectile which needs to hit the target.
Particle			target;													//Target that the projectile has to hit.
float timer = 0.0f;																	// Timer