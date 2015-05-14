#ifndef SPHERE_H
#define SPHERE_H

#include <glm/glm.hpp>
#include "Object3D.h"

using glm::vec3;

class Sphere : public Object3D {
public:
	glm::vec3 center;
	float radius;

	Sphere(glm::vec3 c, float r, glm::vec3 color, Material material)
		:center(c), radius(r), Object3D(color, material)
	{
	}

	virtual void scale(float L) {
		center *= 2 / L;
		center -= vec3(1, 1, 1);
		center.x *= -1;
		center.y *= -1;

		radius *= 2 / L;
	}

	virtual glm::vec3 normal(const glm::vec3& position) {
		return glm::normalize((position - center) / radius);
	}

	bool intersects(vec3 start, vec3 dir, vec3& intersection) {
		vec3 p1 = start + dir;
		float dx = p1.x - start.x;
		float dy = p1.y - start.y;
		float dz = p1.z - start.z;

		float a = dx*dx + dy*dy + dz*dz;
		float b = 2 * dx*(start.x - center.x) + 2 * dy*(start.y - center.y) + 2 * dz*(start.z - center.z);
		float c = center.x*center.x + center.y*center.y + center.z*center.z + start.x*start.x + start.y*start.y + start.z*start.z
			- 2 * (center.x*start.x + center.y*start.y + center.z*start.z) - radius*radius;

		float disc = b*b - 4 * a*c;

		// No intersection
		if (disc < 0) {
			return false;
		}
		// Ray tangent or intersects in two points
		float t = (-b - sqrt(disc)) / (2 * a);
		if (t < 0) {
			return false;
		}
		intersection.x = start.x + t * dx;
		intersection.y = start.y + t * dy;
		intersection.z = start.z + t * dz;

		return true;
	}
};

#endif