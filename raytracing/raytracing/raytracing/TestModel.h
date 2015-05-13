#ifndef TEST_MODEL_CORNEL_BOX_H
#define TEST_MODEL_CORNEL_BOX_H

// Defines a simple test model: The Cornel Box

#include <glm/glm.hpp>
#include <vector>
#include "Object3D.h"
#include "Sphere.h"
#include "Triangle.h"

using glm::vec3;

// Loads the Cornell Box. It is scaled to fill the volume:
// -1 <= x <= +1
// -1 <= y <= +1
// -1 <= z <= +1
void LoadTestModel(std::vector<Object3D*>&  objects)
{
	// Defines colors:
	vec3 red(    0.75f, 0.15f, 0.15f );
	vec3 yellow( 0.75f, 0.75f, 0.15f );
	vec3 green(  0.15f, 0.75f, 0.15f );
	vec3 cyan(   0.15f, 0.75f, 0.75f );
	vec3 blue(   0.15f, 0.15f, 0.75f );
	vec3 purple( 0.75f, 0.15f, 0.75f );
	vec3 white(  0.75f, 0.75f, 0.75f );

	objects.clear();
	objects.reserve( 5*2*3 + 2 );

	// ---------------------------------------------------------------------------
	// Room

	float L = 555;			// Length of Cornell Box side.

	vec3 A(L,0,0);
	vec3 B(0,0,0);
	vec3 C(L,0,L);
	vec3 D(0,0,L);

	vec3 E(L,L,0);
	vec3 F(0,L,0);
	vec3 G(L,L,L);
	vec3 H(0,L,L);

	// Floor:
	objects.push_back(new Triangle(C, B, A, green, Material::Diffuse));
	objects.push_back(new Triangle(C, D, B, green, Material::Diffuse));

	// Left wall
	objects.push_back( new Triangle( A, E, C, purple, Material::Diffuse) );
	objects.push_back( new Triangle( C, E, G, purple, Material::Diffuse) );

	// Right wall
	objects.push_back( new Triangle( F, B, D, yellow, Material::Diffuse) );
	objects.push_back( new Triangle( H, F, D, yellow, Material::Diffuse) );

	// Ceiling
	objects.push_back( new Triangle( E, F, G, cyan, Material::Diffuse) );
	objects.push_back( new Triangle( F, H, G, cyan, Material::Diffuse) );

	// Back wall
	objects.push_back( new Triangle( G, D, C, white, Material::Diffuse) );
	objects.push_back( new Triangle( G, H, D, white, Material::Diffuse) );

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
	objects.push_back(new Triangle(G, F, E, red, Material::Specular));
	objects.push_back(new Triangle(G, H, F, red, Material::Specular));

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
	objects.push_back(new Triangle(E, B, A, blue, Material::Specular));
	objects.push_back(new Triangle(E, F, B, blue, Material::Specular));

	// DOWN
	objects.push_back(new Triangle(F, D, B, blue, Material::Diffuse));
	objects.push_back(new Triangle(F, H, D, blue, Material::Diffuse));

	// BACK
	objects.push_back(new Triangle(H, C, D, blue, Material::Diffuse));
	objects.push_back(new Triangle(H, G, C, blue, Material::Diffuse));

	// LEFT
	objects.push_back(new Triangle(G, E, C, blue, Material::Diffuse));
	objects.push_back(new Triangle(E, A, C, blue, Material::Diffuse));

	// TOP
	objects.push_back(new Triangle(G, F, E, blue, Material::Diffuse));
	objects.push_back(new Triangle(G, H, F, blue, Material::Diffuse));

	// ---------------------------------------------------------------------------
	// Spheres
	// On the short red cube
	objects.push_back(new Sphere(vec3(180, 215, 200), 50, white, Material::Diffuse));

	// On the floor, on the left
	objects.push_back(new Sphere(vec3(480, 50, 100), 50, white, Material::Glass));

	// On the floor, closer to the mirror
	objects.push_back(new Sphere(vec3(330, 50, 170), 50, white, Material::Specular));

	// On the tall blue block
	objects.push_back(new Sphere(vec3(368, 380, 351), 50, white, Material::Specular));

	for (Object3D* o : objects) {
		o->scale(L);
	}
}

#endif