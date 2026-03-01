#include "debug_version.h"

#include <iostream>

void SuiteInfoCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    const char* command = "(apt list -a voxl-suite 2>/dev/null | grep installed | cut -d ' ' -f 2)";
    FILE* pipe = popen(command, "r");

    char buffer[128];
    std::string result = "";

    // Read the command output line by line
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    pclose(pipe); // Close the pipe

    if (strlen(result.c_str()) == 0) {
        command = "(opkg list-installed voxl-suite | sed 's/ - /,/g' | cut -d ',' -f 2)";
        pipe = popen(command, "r");

        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }

        pclose(pipe); // Close the pipe
    }

    mg_ws_send(c, result.c_str(), strlen(result.c_str()), WEBSOCKET_OP_TEXT);
}

void VersionInfoCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
	const char* command = "voxl-version"; // Command to be executed

    FILE* pipe = popen(command, "r"); // Open a pipe to execute the command
    if (!pipe) {
        std::cerr << "Error executing command." << std::endl;
    }

    char buffer[128];
    std::string result = "";

    // Read the command output line by line
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result += buffer;
    }

    pclose(pipe); // Close the pipe

    //std::cout << "Command output:\n" << result << std::endl;
    //printf("\n\nVersion Information\n%s\n\n", result.c_str());
    //printf("\nLength: %lu\n", strlen(result.c_str()));
	mg_ws_send(c, result.c_str(), strlen(result.c_str()), WEBSOCKET_OP_TEXT);
}

