#include <BaremetalCommunicationDef.hpp>

// TODO: is this necessary, Pieter?
/** Instance of the test struct for shared memory. */
// extern TestStruct testComm;

/** Instance of the vision communication struct for shared memory. */
extern VisionCommStruct visionComm;

/** Instance of the QR communication struct for shared memory. */
extern QRCommStruct qrComm;

/** Instance of the logger struct for shared memory. */
extern AccessControlledLogEntry loggerComm;

/**
 * Initialize the structs used for communicating with the Linux core through
 * shared memory.
 */
void initSharedMemoryInstances();