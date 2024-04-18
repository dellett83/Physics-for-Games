#include "DynamicObject.h"
#include "Utility.h"

/*! \brief Brief description.
*  Physics dynamic object class is derived from the GameObject class, as a one type/class of game objects
*  It sets up parameters for physics computation and calls certain physics algorithms.
*  It returns the position and orientation of the object for the visualisation engine to display.
*  It is important to not include any graphics drawings in this class. This is the principle of the separation
*  of physics computation from graphics
*
*/
DynamicObject::DynamicObject()
{
	// Set initial values for physics parameters
	// No forces act on the object to start with
	_force = glm::vec3(0.0f, 0.0f, 0.0f);
	_velocity = glm::vec3(0.0f, 0.0f, 0.0f);

	_mass = 1.0f;
	_bRadius = 1.0f;
	_previous_position = glm::vec3(0.0f);

	//Initialise angular dynamics parameters
	_torque = glm::vec3(0.0f, 0.0f, 0.0f);
	_angular_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	_angular_momentum = glm::vec3(0.0f, 0.0f, 0.0f);

	//Set the rotation matrix to identity matrix
	_R = glm::mat3(1.0f, 0.0f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f);

	global_dampping = 12.0f;

	setMass(1.0f);
}

DynamicObject::~DynamicObject()
{}

void DynamicObject::setMass(float m)
{
	_mass = m;

	setInertiaTensor();

}

void DynamicObject::setInertiaTensor()
{
	glm::mat3 body_inertia;

	body_inertia = glm::mat3{
		(2.0f / 5.0f) * _mass * std::pow(_bRadius, 2),0,0,
		0, (2.0f / 5.0f) * _mass * std::pow(_bRadius, 2),0,
		0,0, (2.0f / 5.0f) * _mass * std::pow(_bRadius,2)
	};

	_body_inertia_tensor_inverse = glm::inverse(body_inertia);

	computeInverseInertiaTensor();
}

void DynamicObject::computeInverseInertiaTensor()
{
	_inertia_tensor_inverse = _R * _body_inertia_tensor_inverse * glm::transpose(_R);
}

glm::vec3 DynamicObject::computeTorque(glm::vec3 torque_arm, glm::vec3 contact_force)
{
	glm::vec3 torque = (glm::cross(contact_force, torque_arm));

	return torque;
}

glm::vec3 DynamicObject::frictionForce(glm::vec3 relative_velocity, glm::vec3 contact_normal, glm::vec3 force_normal, float mu)
{
	glm::vec3 friction_force;
	glm::vec3 forward_relative_velocity = relative_velocity - glm::dot(relative_velocity, contact_normal) * contact_normal;
	float tangent_length = glm::length(forward_relative_velocity);
	if (tangent_length > 1e-6f) //0.000001 (a millionth)
	{
		// normalised vector as direction of travel
		glm::vec3 forward_direction = glm::normalize(forward_relative_velocity);
		//frition direction is opposite of travel
		glm::vec3 friction_direction = -forward_direction;
		friction_force = friction_direction * mu * glm::length(force_normal);
		return friction_force;
	}
	else return glm::vec3(0);
}

void DynamicObject::update(float deltaTs)
{

	/** This function is the work horse of our physics simulation
	* it implements physics simulation algorithm:
	* the basic idea is for each update with a time step size- dt
	* we use F = ma to update the state of every rigid body.
	* Please refer to my lecture notes for more details.
	* The computation consists the following steps
	* Step 1: Clear all the forces act on the object in the previous time step
	* ClearForces();
	* Step 2: Compute all the forces act on the object at the current time step
	* ComputeForces();
	* Step 3: Compute collision response
	* CollisionResponses(dt);
	* Step 4: One step time integration for simulation update !
	* TimeIntegration(dt);
	*/
	
	// Step 1: Clear all the forces act on the object
	clearForces();
	// Step 2: Compute each of forces acts on the object
	// Only gravitational force at the moment
	glm::vec3 force = _mass * _acceleration;
	// Add the force to the total force
	addForce(force);

	// Step 3: Compute collisions and responses
	computeCollisionRes(deltaTs);
	spheresCollisionResponse();

	

	// Step 4:  Integration
	euler(deltaTs); 
	//rk2(deltaTs);  
	//rk4(deltaTs);  
	//verlet(deltaTs);
	
	setPosition(_position.x, _position.y, _position.z);
	
}


void DynamicObject::computeCollisionRes(float deltaTs)
{

	glm::vec3 n = glm::vec3(0.0f, 1.0f, 0.0f);
	glm::vec3 p = _position;
	glm::vec3 q = glm::vec3(0.0f, 0.0f, 0.0f);
	float r = getBoundingRadius();
	float elasticity = 0.9f;

	float d = PFG::DistanceToPlane(n, p, q);
	if (d <= r)
	{
		std::cout << "collided" << std::endl;
		//impulse based response
		glm::vec3 plane_velocity = glm::vec3(0.0f, 0.0f, 0.0f);
		float collision_impulse = -(1 + elasticity) * glm::dot(_velocity - plane_velocity, n) / (1.0f / _mass);
		glm::vec3 collision_impulse_vector = collision_impulse * n;
		_velocity += collision_impulse_vector / _mass;

		//contact force
		glm::vec3 contact_force = glm::vec3(0.0f, 9.8f * _mass, 0.0f);
		addForce(contact_force);
	}

}

/*!\brief: Numerical integration function to compute the current velocity and the current position
* based on the velocity and the position of the previous time step
*/
void  DynamicObject::euler(float deltaTs)
{
	// This function is the numerical integration the dynamic physics computation
	float oneOverMass = 1 / _mass;
	// Compute the current velocity based on the previous velocity
	_velocity += (_force * oneOverMass) * deltaTs;
	// Compute the current position based on the previous position
	_position += _velocity * deltaTs;
}

void DynamicObject::rk2(float deltaTs)
{

	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;

	// Evaluate once at t0
	force = _force;
	acceleration = force / _mass;
	k0 = acceleration * deltaTs;

	// Evaluate once at t0 + deltaT/2.0 using half of k0
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = acceleration * deltaTs;

	// Evaluate once at t0 + deltaT using k1
	_velocity += k1;
	_position += _velocity * deltaTs;
}

void DynamicObject::rk4(float deltaTs)
{

	glm::vec3 force;
	glm::vec3 acceleration;
	glm::vec3 k0;
	glm::vec3 k1;
	glm::vec3 k2;
	glm::vec3 k3;

	// Evaluate once at t0 to find k0
	force = _force;
	acceleration = force / _mass;
	k0 = deltaTs * acceleration;

	// Evaluate twice at t0 + deltaT/2.0 using half of k0 and half of k1
	force = _force + k0 / 2.0f;
	acceleration = force / _mass;
	k1 = deltaTs * acceleration;

	force = _force + k1 / 2.0f;
	acceleration = force / _mass;
	k2 = deltaTs * acceleration;

	// Evaluate once at t0 + deltaT using k2
	force = _force + k2;
	acceleration = force / _mass;
	k3 = deltaTs * acceleration;

	// Evaluate at t0 + deltaT using weighted sum of k0, k1, k2, and k3
	_velocity += (k0 + 2.0f * k1 + 2.0f * k2 + k3) / 6.0f;
	// Update position
	_position += _velocity * deltaTs;
}

void DynamicObject::verlet(float deltaTs)
{   /* Equations for your reference 
	*https://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
	*/
	glm::vec3 acceleration = _force / _mass;
	_previous_position = _position - _velocity * deltaTs + 0.5f * acceleration * deltaTs * deltaTs;
	_position = 2.0f * _position - _previous_position + acceleration * deltaTs * deltaTs;
	_velocity = (_position - _previous_position) / (2.0f * deltaTs);
	_velocity += acceleration * deltaTs;

}

void DynamicObject::spheresCollisionResponse()
{
	//safely converts pointers and references to classes up down and sideways along the inheritence hierarchy
	for (int i = 0; i < m_dynamicObjects.size(); i++)
	{
		DynamicObject* _other = m_dynamicObjects[i];
		float elasticity = 0.66f;
		glm::vec3 position_distance = _position - _other->getPosition();
		glm::vec3 n = glm::normalize(position_distance);
		float r1 = _bRadius;
		float r2 = _other->getBoundingRadius();
		glm::vec3 other_velocity = _other->getVelocity();
		glm::vec3 velA = _velocity;
		glm::vec3 velB = other_velocity;
		glm::vec3 relative_velocity = velA - velB;
		glm::vec3 cp;

		float distance = glm::length(position_distance);
		bool collision = PFG::SphereToSphereCollision(_position, _other->getPosition(), r1, r2, cp);
		if (collision)
		{
			applyImpulseResponses(this, _other);
			float penetration = abs((r1 + r2) - distance);
			float _mass2 = _other->getMass();
			float inverse_mass1 = 1.0f / _mass;
			float inverse_mass2 = 1.0f / _mass2;
			float total_inverse_mass = inverse_mass1 + inverse_mass2;
			float total_mass = _mass + _mass2;

			glm::vec3 mov = _position + penetration * n * inverse_mass1 / total_inverse_mass;
			setPosition(mov.x, mov.y, mov.z);
			mov = _other->getPosition() + penetration * n * inverse_mass2 / total_inverse_mass;
			_other->setPosition(mov.x, mov.y, mov.z);
		}
	}
}

void DynamicObject::applyImpulseResponses(DynamicObject* objA, DynamicObject* objB)
{
	glm::vec3 position_distance = objA -> getPosition() - objB->getPosition();
	glm::vec3 n = glm::normalize(position_distance);
	float rA = objA->getBoundingRadius();
	float rB = objB->getBoundingRadius();
	glm::vec3 velocity_A = objA->getVelocity();
	glm::vec3 velocity_B = objB->getVelocity();
	glm::vec3 relative_velocity = velocity_A - velocity_B;

	//multiple restitutions of the two objects

	float elasticity = 0.6f * 0.5f;
	float one_over_massA = 1.0f / objA->getMass();
	float one_over_massB = 1.0f / objB->getMass();
	float J_numerator = -(1.0f + elasticity) * glm::dot(relative_velocity, n);
	float total_inverse_mass = one_over_massA + one_over_massB;
	float J = J_numerator / (total_inverse_mass);

	glm::vec3 collision_impulse_vector = J * n;
	//obj A
	glm::vec3 velocity = objA->getVelocity() + collision_impulse_vector * one_over_massA;
	objA->setVelocity(velocity);

	//obj B
	velocity = objB->getVelocity() - collision_impulse_vector * one_over_massB;
	objB->setVelocity(velocity);
}