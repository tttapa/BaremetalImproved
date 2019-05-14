#pragma once

/* Includes from src. */
#include <BaremetalCommunicationDef.hpp>
#include <LogEntry.h>
#include <SharedStruct.hpp>

/** Instance of the test struct for shared memory. */
volatile extern TestStruct *testComm;

/** Instance of the vision communication struct for shared memory. */
volatile extern VisionCommStruct *visionComm;

/** Instance of the QR communication struct for shared memory. */
volatile extern QRCommStruct *qrComm;

/** Instance of the logger struct for shared memory. */
volatile extern AccessControlledLogEntry *loggerComm;

/**
 * Initialize the structs used for communicating with the Linux core through
 * shared memory.
 */
void initSharedMemoryInstances();