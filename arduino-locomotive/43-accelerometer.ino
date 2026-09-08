// ================================================================================================
// File description
// ================================================================================================
// Accelerometer stub. This revision of the train has no accelerometer wired in; the full custom driver
// and tilt/crash safety logic are preserved in backup\43-accelerometer-custom.ino for a future
// hardware revision. To keep other tabs compiling unchanged, this file provides no-op stand-ins
// for the four accelerometer entry points forward-declared in arduino-locomotive.ino and reports
// the accelerometer as "detected" so the boot-time error flag (ERR_ACCELEROMETER, bit 3) does
// not fire on every startup.

bool accelerometerDetected = true;  // No hardware present; report OK so no boot fault is raised.

void initAccelerometerHardware() {}
void updateAccelerometerSafety() {}
void sleepAccelerometer() {}
void wakeAccelerometer() {}
