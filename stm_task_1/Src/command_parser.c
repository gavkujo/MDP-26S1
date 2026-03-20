#include "command_parser.h"
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static inline int isdig_uc(char c){ return isdigit((unsigned char)c); }

/**
 * @brief Parse a 7-character command string from RPi
 * Format: "DDCCCCC" where DD are digits, CCCCC is the 5-char command
 * Examples: "00FW090", "00FR090", "00FL090", "00STOP-", "00FW---", "00BW075",
 *           "00BL090", "00BR090"
 */
bool Command_Parse(const char* s, RobotCommand_t* out) {
    if (!s || !out) return false;

    if (!Command_ValidateFormat(s)) {
        out->cmd_type = CMD_INVALID;
        return false;
    }

    // detection (2 digits)
    out->detection_data[0] = s[0];
    out->detection_data[1] = s[1];
    out->detection_data[2] = '\0';

    // command part: indices 2..6 (5 chars)
    char cmd_part[6];
    memcpy(cmd_part, &s[2], 5);
    cmd_part[5] = '\0';

    out->units = 0;
    out->is_indefinite = false;
    out->cmd_type = CMD_INVALID;

    // fixed patterns
    if (!strncmp(cmd_part, "STOP-", 5)) { out->cmd_type = CMD_STOP; return true; }

    // forward/back indefinite
    if (!strncmp(cmd_part, "FW---", 5)) { out->cmd_type = CMD_FORWARD_INDEFINITE;  out->is_indefinite = true; return true; }
    if (!strncmp(cmd_part, "BW---", 5)) { out->cmd_type = CMD_BACKWARD_INDEFINITE; out->is_indefinite = true; return true; }

    // forward 90° turns
    if (!strncmp(cmd_part, "FL090", 5)) { out->cmd_type = CMD_TURN_LEFT_90;  return true; }
    if (!strncmp(cmd_part, "FR090", 5)) { out->cmd_type = CMD_TURN_RIGHT_90; return true; }

    // NEW: backward 90° turns
    if (!strncmp(cmd_part, "BL090", 5)) { out->cmd_type = CMD_TURN_LEFT_90_BACK;  return true; }
    if (!strncmp(cmd_part, "BR090", 5)) { out->cmd_type = CMD_TURN_RIGHT_90_BACK; return true; }

    // FWddd / BWddd (distance with 3 digits)
    if ((!strncmp(cmd_part, "FW", 2) || !strncmp(cmd_part, "BW", 2)) &&
        isdig_uc(cmd_part[2]) && isdig_uc(cmd_part[3]) && isdig_uc(cmd_part[4])) {

        char u[4] = { cmd_part[2], cmd_part[3], cmd_part[4], '\0' };
        out->units = (uint16_t)atoi(u);
        out->cmd_type = (cmd_part[0] == 'F') ? CMD_FORWARD : CMD_BACKWARD;
        return true;
    }

    out->cmd_type = CMD_INVALID;
    return false;
}

/**
 * @brief Validate if command string has correct 7-char format
 * Robust to trailing CR/LF/spaces: trims them before checking length.
 */
bool Command_ValidateFormat(const char* s) {
    if (!s) return false;

    // compute visible length up to CR/LF/NUL, then trim trailing spaces
    size_t n = 0;
    while (s[n] != '\0' && s[n] != '\r' && s[n] != '\n') n++;
    while (n > 0 && s[n-1] == ' ') n--;

    if (n != 7) return false;                         // require exactly 7 visible chars
    if (!isdig_uc(s[0]) || !isdig_uc(s[1])) return false; // first two digits

    // command part 5 chars
    char c[6];
    memcpy(c, &s[2], 5);
    c[5] = '\0';

    // fixed patterns
    if (!strcmp(c, "STOP-")) return true;
    if (!strcmp(c, "FW---")) return true;
    if (!strcmp(c, "BW---")) return true;
    if (!strcmp(c, "FL090")) return true;
    if (!strcmp(c, "FR090")) return true;

    // NEW: backward 90° turns
    if (!strcmp(c, "BL090")) return true;
    if (!strcmp(c, "BR090")) return true;

    // FWddd / BWddd
    if ( (c[0]=='F' && c[1]=='W' && isdig_uc(c[2]) && isdig_uc(c[3]) && isdig_uc(c[4])) ||
         (c[0]=='B' && c[1]=='W' && isdig_uc(c[2]) && isdig_uc(c[3]) && isdig_uc(c[4])) ) {
        return true;
    }

    return false;
}

const char* Command_GetTypeString(CommandType_t t) {
    switch (t) {
        case CMD_FORWARD:               return "FORWARD";
        case CMD_BACKWARD:              return "BACKWARD";
        case CMD_STOP:                  return "STOP";
        case CMD_FORWARD_INDEFINITE:    return "FORWARD_INDEFINITE";
        case CMD_BACKWARD_INDEFINITE:   return "BACKWARD_INDEFINITE";
        case CMD_TURN_LEFT_90:          return "TURN_LEFT_90";
        case CMD_TURN_RIGHT_90:         return "TURN_RIGHT_90";
        case CMD_TURN_LEFT_90_BACK:     return "TURN_LEFT_90_BACK";   // NEW
        case CMD_TURN_RIGHT_90_BACK:    return "TURN_RIGHT_90_BACK";  // NEW
        case CMD_INVALID:               return "INVALID";
        default:                        return "UNKNOWN";
    }
}
