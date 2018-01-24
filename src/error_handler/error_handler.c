#include "includes.h"

void error_handler(errorMask_t error)
{
    (void)(error);
    while(1);
}