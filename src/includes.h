#pragma once

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

//target specific
#include "default.h"

//stm32 defines, anything that uses std per, trying to separate all driver specific code from program logic
#include "reset.h"
#include "flash.h"
#include "isr_priority.h"
#include "config.h"
#include "clock.h"
#include "gpio.h"
#include "boothandler.h"
#include <arm_math.h>
