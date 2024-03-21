#include "phy.h"
#include "glm/glm.hpp"

#include <cmath>
#include "KinematicsObject.h"
#include "DynamicObject.h"
#include "Utility.h"

int main()
{
	DynamicObject a;
	DynamicObject b;
	DynamicObject c;
	DynamicObject d;

	float dt = 1.0f / 60.0f;

	//set floor
	a.setType(PHY_SQUARE);
	a.setColor(1, 1, 1);
	a.setScale(100, 100,100);
	a.setPosition(0.0f,0.0f,0.0f);
	a.setVelocity(glm::vec3(0.0f, 0.0f, 0.0f));
	a.setRotation(-90.0f, 0.0f, 0.0f);
	a.setMass(10000000000000000000000.0f);

	b.setType(PHY_SPHERE);
	b.setColor(1, 0, 1);
	b.setScale(8, 8, 8);
	b.setPosition(0, 15, 0);
	b.setVelocity(glm::vec3(2.0f, 0.0f, 2.0f));
	b.setAcceleration(glm::vec3(0.0f, -9.8f, 0.0f));
	b.setBoundingRadius(4.0f);
	b.setMass(5.0f);

	c.setType(PHY_SPHERE);
	c.setColor(1, 1, 0);
	c.setScale(6, 6, 6);
	c.setPosition(10, 10, 0);
	c.setVelocity(glm::vec3(-1, 0, 0));
	c.setAcceleration(glm::vec3(0.0f, -9.8f, 0.0f));
	c.setBoundingRadius(3.0f);
	c.setMass(10.0f);

	d.setType(PHY_SPHERE);
	d.setColor(0, 1, 0);
	d.setScale(10.0f,10.0f,10.0f);
	d.setPosition(10.0f, 20.0f, 0.0f);
	d.setVelocity(glm::vec3(2.0f, 0.0f, 0.0f));
	d.setAcceleration(glm::vec3(0.0f, -9.8f, 0.0f));
	d.setBoundingRadius(5.0f);
	d.setMass(20.0f);

	b.setCollisionObject(&c);
	b.setCollisionObject(&d);
	c.setCollisionObject(&b);
	c.setCollisionObject(&d);
	d.setCollisionObject(&c);
	d.setCollisionObject(&b);

	while (true)
	{
		b.update(dt);
		c.update(dt);
		d.update(dt);

		wait(5);
	}

	return 0;
}
