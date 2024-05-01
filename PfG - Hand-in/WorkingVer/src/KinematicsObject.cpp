
#include "KinematicsObject.h"


/*! \brief Brief description.
*  Physics object class is derived from the GameObject class, as a one type/class of game objects
*  It sets up parameters for physics computation and calls certain physics algorithms.
*  It returns the position and orientation of the object for the visualisation engine to display.
*  It is important to not include any graphics drawings in this class. This is the principle of the separation
*  of physics computation from graphics
*
*/

KinematicsObject::KinematicsObject()
{
	// Set initial values for parameters
	 _velocity = glm::vec3(0.0f, 0.0f, 0.0f);
	 _position = glm::vec3(0.0f, 0.0f, 0.0f);
	 _acceleration = glm::vec3(0.0f, 0.0f, 0.0f);
}

KinematicsObject::~KinematicsObject(){}

void KinematicsObject::update(float deltaTs)
{
		glm::vec3 vel;
		_position.x += _velocity.x * deltaTs + 0.5f * (_acceleration.x) * deltaTs * deltaTs;
		_position.z += _velocity.z * deltaTs + 0.5f * (_acceleration.z) * deltaTs * deltaTs;
		_position.y += _velocity.y * deltaTs + 0.5f * (_acceleration.y) * deltaTs * deltaTs;
		vel.x = _velocity.x + (_acceleration.x) * deltaTs;
		vel.y = _velocity.y + (_acceleration.y) * deltaTs;
		vel.z = _velocity.z + (_acceleration.z) * deltaTs;
		_velocity = vel; // update the initial velocity

		// collision detection
		if (_position.y <= 3.0f)
			_position.y = 3.0f;

		setPosition(_position.x, _position.y, _position.z);
	}

     


