#ifndef TEST_MODEL_CORNEL_BOX_H
#define TEST_MODEL_CORNEL_BOX_H

#include <glm/glm.hpp>
#include <vector>
#include "Object3D.h"
#include "Sphere.h"
#include "Triangle.h"

// Cornell Box model

using glm::vec3;

/* Loads the Cornell Box, then scale it to fill the volume:
 * -1 <= x <= +1
 * -1 <= y <= +1
 * -1 <= z <= +1
 *
 * Axis:
 * - Before scaling:
 *		- Parnell box size: 555
 *		- Origin: (right, bottom, front)
 *		- Directions: (x: left, y: top: z: deepness)
 * - After scaling:
 *		- Parnell box size: 2
 *		- Origin: center of the scene on the three axis
 *		- Directions: (x: right, y: down: z: deepness)
 */
void LoadTestModel(std::vector<Object3D*>&  objects) {
	// Colors
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.25f, 0.60f, 0.0f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.05f, 0.6f, 1.0f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 beige(0.85f, 0.85f, 0.7f);
	vec3 mauve(0.6f, 0.45f, 0.9f);
	vec3 white(0.75f, 0.75f, 0.75f);
	vec3 orange(0.8f, 0.7f, 0.05f);

	objects.clear();
	objects.reserve( 5*2*3 + 2 );

	// ---------------------------------------------------------------------------
	// Room

	float L = 555;			// Length of Cornell Box side.

	vec3 A(L,0,0);
	vec3 B(0,0,0);
	vec3 C(L,0,L);
	vec3 D(0,0,L);

	vec3 E(L,L-1,0);
	vec3 F(0, L - 1, 0);
	vec3 G(L, L - 1, L);
	vec3 H(0, L - 1, L);

	// Floor:
	objects.push_back(new Triangle(C, B, A, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(C, D, B, beige, Material::DiffuseSpecular));

	// Left wall
	objects.push_back(new Triangle(A, E, C, red, Material::DiffuseSpecular));
	objects.push_back(new Triangle(C, E, G, red, Material::DiffuseSpecular));

	// Right wall
	objects.push_back(new Triangle(F, B, D, blue, Material::DiffuseSpecular));
	objects.push_back(new Triangle(H, F, D, blue, Material::DiffuseSpecular));

	// Back wall
	objects.push_back(new Triangle(G, D, C, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(G, H, D, beige, Material::DiffuseSpecular));

	// Ceiling
	// Full ceiling
	// objects.push_back(new Triangle(E, F, G, beige, Material::DiffuseSpecular));
	// objects.push_back(new Triangle(F, H, G, beige, Material::DiffuseSpecular));

	// Ceiling with a hole for light
	int holeRadius = 75;
	vec3 I(L / 2 + holeRadius, L, L / 2 - holeRadius);
	vec3 J(L / 2 - holeRadius, L, L / 2 - holeRadius);
	vec3 K(L / 2 + holeRadius, L, L / 2 + holeRadius);
	vec3 L2(L / 2 - holeRadius, L, L / 2 + holeRadius);

	vec3 M(L / 2 + holeRadius, L, 0);
	vec3 N(L / 2 - holeRadius, L, 0);
	vec3 O(L / 2 + holeRadius, L, L+5);
	vec3 P(L / 2 - holeRadius, L, L + 5);
	E = vec3(L + 5, L, 0);
	F = vec3(-5, L, 0);
	G = vec3(L+5, L, L + 5);
	H = vec3(-5, L, L + 5);

	objects.push_back(new Triangle(E, M, G, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(M, O, G, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(M, N, I, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(N, J, I, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(N, F, P, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(F, H, P, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(K, L2, O, beige, Material::DiffuseSpecular));
	objects.push_back(new Triangle(L2, P, O, beige, Material::DiffuseSpecular));

	// Small rectangles around the hole in the ceiling
	int lightBoxHeight = 5;
	M = vec3(L / 2 + holeRadius, L - lightBoxHeight, L / 2 - holeRadius);
	N = vec3(L / 2 - holeRadius, L - lightBoxHeight, L / 2 - holeRadius);
	O = vec3(L / 2 + holeRadius, L - lightBoxHeight, L / 2 + holeRadius);
	P = vec3(L / 2 - holeRadius, L - lightBoxHeight, L / 2 + holeRadius);

	objects.push_back(new Triangle(I, J, M, beige, Material::Specular));
	objects.push_back(new Triangle(J, N, M, beige, Material::Specular));
	objects.push_back(new Triangle(J, L2, N, beige, Material::Specular));
	objects.push_back(new Triangle(L2, P, N, beige, Material::Specular));
	objects.push_back(new Triangle(L2, K, O, beige, Material::Specular));
	objects.push_back(new Triangle(L2, O, P, beige, Material::Specular));
	objects.push_back(new Triangle(I, M, O, beige, Material::Specular));
	objects.push_back(new Triangle(K, I, O, beige, Material::Specular));

	// ---------------------------------------------------------------------------
	// Short block

	A = vec3(290,0,114);
	B = vec3(130,0, 65);
	C = vec3(240,0,272);
	D = vec3( 82,0,225);

	E = vec3(290,165,114);
	F = vec3(130,165, 65);
	G = vec3(240,165,272);
	H = vec3( 82,165,225);

	// Front
	objects.push_back(new Triangle(E, B, A, red, Material::Glass));
	objects.push_back(new Triangle(E, F, B, red, Material::Glass));

	// Right
	objects.push_back(new Triangle(F, D, B, red, Material::Glass));
	objects.push_back(new Triangle(F, H, D, red, Material::Glass));

	// BACK
	objects.push_back(new Triangle(H, C, D, red, Material::Glass));
	objects.push_back(new Triangle(H, G, C, red, Material::Glass));

	// LEFT
	objects.push_back(new Triangle(G, E, C, red, Material::Glass));
	objects.push_back(new Triangle(E, A, C, red, Material::Glass));

	// TOP
	objects.push_back(new Triangle(G, F, E, red, Material::Glass));
	objects.push_back(new Triangle(G, H, F, red, Material::Glass));

	// ---------------------------------------------------------------------------
	// Tall block

	A = vec3(423,0,247);
	B = vec3(265,0,296);
	C = vec3(472,0,406);
	D = vec3(314,0,456);

	E = vec3(423,330,247);
	F = vec3(265,330,296);
	G = vec3(472,330,406);
	H = vec3(314,330,456);

	// Front
	objects.push_back(new Triangle(E, B, A, green, Material::Specular));
	objects.push_back(new Triangle(E, F, B, green, Material::Specular));

	// DOWN
	objects.push_back(new Triangle(F, D, B, green, Material::Diffuse));
	objects.push_back(new Triangle(F, H, D, green, Material::Diffuse));

	// BACK
	objects.push_back(new Triangle(H, C, D, green, Material::Diffuse));
	objects.push_back(new Triangle(H, G, C, green, Material::Diffuse));

	// LEFT
	objects.push_back(new Triangle(G, E, C, green, Material::Diffuse));
	objects.push_back(new Triangle(E, A, C, green, Material::Diffuse));

	// TOP
	objects.push_back(new Triangle(G, F, E, green, Material::Diffuse));
	objects.push_back(new Triangle(G, H, F, green, Material::Diffuse));

	// ---------------------------------------------------------------------------
	// Spheres
	// On the short red cube
	objects.push_back(new Sphere(vec3(180, 215, 200), 50, white, Material::Glass));

	// On the floor, closer to the mirror
	objects.push_back(new Sphere(vec3(330, 50, 170), 50, white, Material::Specular));

	// On the tall blue block
	objects.push_back(new Sphere(vec3(368, 380, 351), 50, white, Material::Specular));

	// On the floor, on the left
	objects.push_back(new Sphere(vec3(470, 50, 100), 50, orange, Material::Diffuse));

	// Small sphere in front of the bigger one which is on the floor, on the left
	objects.push_back(new Sphere(vec3(515, 20, 25), 20, white, Material::Glass));

	// Right in the wall
	objects.push_back(new Sphere(vec3(0, L / 2, L / 2), 90, white, Material::Specular));

	for (Object3D* o : objects) {
		o->scale(L);
	}
}

#endif
