#ifndef OBJECT3D_H
#define OBJECT3D_H

#include <glm/glm.hpp>
#include "Material.h"

using glm::vec3;

class Object3D {
public:
	vec3 color;
	Material material;

	Object3D(vec3 color, Material material)
		:color(color), material(material) {
	}

	/* Return the normal of the current object. The angle between the normal and the oppositeDir
	 * vector will be higher than 90 degrees. position is only required for spheres */
	virtual glm::vec3 normal(const glm::vec3& position) = 0;

	virtual bool intersects(vec3 start, vec3 dir, vec3& intersection) = 0;

	/* Scale to the volume [-1,1]^3 */
	virtual void scale(float L) = 0;
};
#endif