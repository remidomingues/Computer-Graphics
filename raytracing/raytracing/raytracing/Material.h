#ifndef MATERIAL_H
#define MATERIAL_H

float AIR_REFRACTIVE_INDEX = 1.0;
float GLASS_REFRACTIVE_INDEX = 1.52;
float DIFFUSE_SPECULAR_REFLECTION = 0.2;

typedef enum Material {
	Diffuse,
	DiffuseSpecular,
	Specular,
	Glass
};

#endif