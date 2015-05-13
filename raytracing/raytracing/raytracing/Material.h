#ifndef MATERIAL_H
#define MATERIAL_H

float AIR_REFRACTIVE_INDEX = 1.0;
float GLASS_REFRACTIVE_INDEX = 1.5;
float GLASS_REFRACTED_LIGHT = 0.9;

typedef enum Material {
	Diffuse,
	Specular,
	Glass
};

#endif