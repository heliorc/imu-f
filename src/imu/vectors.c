#include "includes.h"
#include "vectors.h"

void VectorCrossProduct(vector_record_t *vectorOut, vector_record_t *vectorIn1, vector_record_t *vectorIn2)
{ 
    vectorOut->x = vectorIn1->y * vectorIn2->z - vectorIn1->z * vectorIn2->y;
    vectorOut->y = vectorIn1->z * vectorIn2->x - vectorIn1->x * vectorIn2->z;
    vectorOut->z = vectorIn1->x * vectorIn2->y - vectorIn1->y * vectorIn2->x;
}

void VectorAddVector(vector_record_t *vectorOut, vector_record_t *vectorIn, float trust)
{
    vectorOut->x += vectorIn->x * trust;
    vectorOut->y += vectorIn->y * trust;
    vectorOut->z += vectorIn->z * trust;
}

void VectorZeroVector(vector_record_t *vector)
{
	vector->x = 0.0f;
	vector->y = 0.0f;
	vector->z = 0.0f;
}