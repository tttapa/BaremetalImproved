#include <SharedMemoryInstances.hpp>

/** Instance of the test struct for shared memory. */
volatile TestStruct *testComm;

/** Instance of the vision communication struct for shared memory. */
volatile VisionCommStruct *visionComm;

/** Instance of the QR communication struct for shared memory. */
volatile QRCommStruct *qrComm;

/** Instance of the logger struct for shared memory. */
volatile AccessControlledLogEntry *loggerComm;

void initSharedMemoryInstances() {
    testComm   = TestStruct::init();
    visionComm = VisionCommStruct::init();
    qrComm     = QRCommStruct::init();
    loggerComm = AccessControlledLogEntry::init();
}