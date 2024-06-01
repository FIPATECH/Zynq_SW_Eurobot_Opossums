#include <stdio.h>
#include <xil_io.h>
#include <xparameters.h>
#include <math.h>

#include "platform.h"
#include "xil_printf.h"
#include "xgpio.h"
#include "xscutimer.h"
#include "xil_exception.h"
#include "xparameters_ps.h"
#include "xscugic.h"
#include "xcanps.h"

#include "QEI.h"
#include "Timer.h"
#include "Interrupt.h"
#include "Asserv_Loop.h"
#include "PWM.h"
#include "CAN.h"

#include "xscugic.h"

//#define DEBUG
