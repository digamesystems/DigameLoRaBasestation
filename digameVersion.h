#ifndef __DIGAME_VERSION_H__
#define __DIGAME_VERSION_H__

const String SW_VERSION       = "0.9.90";
const String TERSE_SW_VERSION = "0990";  

/*
 * 0.9.88 - Updates to support refactored digameDisplay.h 
 * 0.9.89 - Made the decision to limit web UI to AP Mode. (Network conficts when vehicle events happen in the middle of operation.)
 * 0.9.90 - Add support for MQTT. Improve message parsing / error checking by adding a check for well-formed messages before adding 
 *          them to the message queue for processing.
 */

#endif
