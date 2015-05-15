#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <glm/glm.hpp>
#include "Material.h"

using glm::vec3;

// Describe a triangle or a sphere
class Object3D {
public:
	// Object color (relevant if material is diffuse, diffuse and specular
	// or if the maximal number of bounces has been reached
	vec3 color;
	// Object material
	Material material;

	/* Constructor */
	Object3D(vec3 color, Material material)
		:color(color), material(material) {
	}

	/* Return the normal of the current object. The position (of an intersection ray / object)
	 * is relevant only for calls applied to a sphere */
	virtual glm::vec3 normal(const glm::vec3& position) = 0;

	/* Return true and initialize the intersection parameter if the ray defined by
	 * the starting point and direction intersects the objects, false otherwise */
	virtual bool intersects(vec3 start, vec3 dir, vec3& intersection) = 0;

	/* Scale to the volume [-1,1]^3 */
	virtual void scale(float L) = 0;
};

#endif
