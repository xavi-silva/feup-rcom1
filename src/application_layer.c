// Application layer protocol implementation

#include "../include/application_layer.h"
#include "../include/link_layer.h"

#include <stdio.h>

void applicationLayer(const char *serialPort, const char *role, int baudRate,
                      int nTries, int timeout, const char *filename)
{
    LinkLayer connectionParameters = {*serialPort, *role, baudRate, nTries, timeout};
    
    llopen(connectionParameters);
}

