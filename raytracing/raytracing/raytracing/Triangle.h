#ifndef TRIANGLE_H
#define TRIANGLE_H

#include <glm/glm.hpp>
#include "Object3D.h"

using glm::vec3;

// Used to describe a triangular surface:
class Triangle : public Object3D {
public:
	glm::vec3 v0;
	glm::vec3 v1;
	glm::vec3 v2;
	glm::vec3 _normal;

	Triangle(glm::vec3 v0, glm::vec3 v1, glm::vec3 v2, glm::vec3 color, Material material)
		: v0(v0), v1(v1), v2(v2), Object3D(color, material) {
		ComputeNormal();
	}

	/* Return x (t u v) from ray start and direction. t distance, u and v triangle delimiters */
	virtual bool intersects(vec3 start, vec3 dir, vec3& intersection) {
		float EPSILON = 0.000001;
		vec3 P, Q, T;
		float det, inv_det, u, v;
		float t;

		// Edges sharing v0
		vec3 e1 = v1 - v0;
		vec3 e2 = v2 - v0;
		// Compute determinant
		P = glm::cross(dir, e2);
		det = glm::dot(e1, P);

		if (det > -EPSILON && det < EPSILON) {
			// The ray lies within the plane of the triangle
			return 0;
		}
		inv_det = 1.f / det;

		// Calculate the distance from v0 to the start point
		T = start - v0;

		// Calculate u parameter and test bound
		u = glm::dot(T, P) * inv_det;
		if (u < 0.f || u > 1.f) {
			// The intersection lies outside of the triangle
			return 0;
		}

		// V parameter
		Q = glm::cross(T, e1);
		v = glm::dot(dir, Q) * inv_det;

		if (v < 0.f || u + v  > 1.f) {
			// The intersection lies outside of the triangle
			return 0;
		}

		t = glm::dot(e2, Q) * inv_det;

		if (t > EPSILON) {
			// Ray intersection
			intersection = start + t * dir;
			return 1;
		}

		// No intersection
		return 0;
	}

	virtual glm::vec3 normal(const glm::vec3& oppositeDir, const glm::vec3& position) {
		return _normal;
	}

	virtual void scale(float L) {
		v0 *= 2 / L;
		v1 *= 2 / L;
		v2 *= 2 / L;

		v0 -= vec3(1, 1, 1);
		v1 -= vec3(1, 1, 1);
		v2 -= vec3(1, 1, 1);

		v0.x *= -1;
		v1.x *= -1;
		v2.x *= -1;

		v0.y *= -1;
		v1.y *= -1;
		v2.y *= -1;

		ComputeNormal();
	}

private:
	void ComputeNormal()
	{
		glm::vec3 e1 = v1 - v0;
		glm::vec3 e2 = v2 - v0;
		_normal = glm::normalize(glm::cross(e2, e1));
	}
};
#endif