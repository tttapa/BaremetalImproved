#pragma once

/**
 * @todo    Better name.
 * 
 * You shouldn't do any heavy calculations in this main loop, it just checks the
 * interrupt timing. It turns on the first on-board LED when the code in the 
 * interrupt is too slow.
 */
void mainLoop();