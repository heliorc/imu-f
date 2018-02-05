#pragma once

typedef struct vector_record {
	volatile float x;
	volatile float y;
	volatile float z;
} vector_record_t;

extern void VectorCrossProduct(vector_record_t *vectorOut, vector_record_t *vectorIn1, vector_record_t *vectorIn2);
extern void VectorAddVector(vector_record_t *vectorOut, vector_record_t *vectorIn, float trust);
extern void VectorZeroVector(vector_record_t *vector);