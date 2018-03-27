#include "includes.h"
#include "error_handler.h"

void error_handler(errorMask_t error)
{
    (void)(error);
    while(1);
}