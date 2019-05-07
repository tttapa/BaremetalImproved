#include <SharedMemoryInstances.hpp>

// TODO: is this necessary, Pieter?
/** Instance of the test struct for shared memory. */
// TestStruct testComm;

/** Instance of the vision communication struct for shared memory. */
VisionCommStruct visionComm;

/** Instance of the QR communication struct for shared memory. */
QRCommStruct qrComm;

/** Instance of the logger struct for shared memory. */
AccessControlledLogEntry loggerComm;

void initSharedMemoryInstances() {
    // TODO: these don't exist??
    //testComm   = TestStruct::init();
    //visionComm = VisionCommStruct::init();
    //qrComm     = QRCommStruct::init();
    //loggerComm = AccessControlledLogEntry::init();
}