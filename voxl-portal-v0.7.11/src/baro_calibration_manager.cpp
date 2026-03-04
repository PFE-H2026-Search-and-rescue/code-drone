#include "baro_calibration_manager.h"

#include <modal_pipe.h>
#include <stdlib.h>
#include <voxl_cutils.h>
#include <turbojpeg.h>
#include <iostream>

#include "cmd_group_manager.h"
#include "manager_utils.h"
#include "page_manager.h"

extern struct mg_mgr mgr_;
static websocket_info ws_info;
volatile bool should_continue = false;

volatile bool done_reaching_temp = false;

static uint8_t current_sysid = 1;

static uint8_t autopilot_monitor_get_sysid(void)
{
	return current_sysid;
}

void BaroCalibrationCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data) {
    if (c == nullptr) {
        fprintf(stderr, "Error reading from barometer calibration callback\n");
        return;
    }

    if (c->is_closing)
    {
        // Remove this connection from our connection ids
        ws_info.connection_ids.erase(std::remove(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id), ws_info.connection_ids.end());

        // Close the pipe if no longer sending to any websockets
        if (ws_info.connection_ids.empty())
        {
            ws_info.connected = false;
        }
        return;
    }

    if (!ws_info.connected)
    {
        ws_info.connected = true;
    }

    ws_info.connection_ids.push_back(c->id);
}

void send_baro_calibration_params(struct baro_calibration_params baroCalParamsMsg) {

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next){
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
            continue;
        }

        mg_ws_send(c, (char *)&baroCalParamsMsg, sizeof(struct baro_calibration_params), WEBSOCKET_OP_BINARY);
    }
}


void send_baro_calibration_stage(uint8_t calibration_stage, uint16_t pos=0) {
    struct baro_calibration_progress baroProgressMsg;
    baroProgressMsg.msg_index = 3;
    baroProgressMsg.pos = pos;
    baroProgressMsg.calibration_stage = calibration_stage;

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next){
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
            continue;
        }

        mg_ws_send(c, (char *)&baroProgressMsg, sizeof(struct baro_calibration_progress), WEBSOCKET_OP_BINARY);
    }
}

void send_baro_calibration_progress(uint16_t pos) {
    struct baro_calibration_progress baroProgressMsg;
    baroProgressMsg.msg_index = 3;
    baroProgressMsg.pos = pos;
    baroProgressMsg.calibration_stage = 0;

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next){
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
            continue;
        }

        mg_ws_send(c, (char *)&baroProgressMsg, sizeof(struct baro_calibration_progress), WEBSOCKET_OP_BINARY);
    }
}

void save_baro_calibration_params() {
	pthread_t thread_id;
	pthread_create(&thread_id, nullptr, save_baro_calibration_params_helper, nullptr);
}

void save_baro_calibration_params_helper() {
    pid_t pid;

    char *argv[] = {
        "/usr/bin/voxl-configure-px4-params",
        "-n",
        "-f",
        "/data/px4/param/parameters_baro_tc.cal",
        NULL
    };

    // Fork a child process
    pid = fork();
    fprintf(stderr, "Process id %d\n", pid);

    if (pid == 0) {
        execv(argv[0], argv);
    }

    int status;
    waitpid(pid, &status, 0);


    send_save_params_success();
}

void send_save_params_success() {
    save_params_success success_msg;
    success_msg.msg_index = 4;
    success_msg.success = 1;

    struct mg_connection *c;
    for (c = mgr_.conns; c != NULL; c = c->next){
        if (!(std::count(ws_info.connection_ids.begin(), ws_info.connection_ids.end(), c->id))){
            continue;
        }

        mg_ws_send(c, (char *)&success_msg, sizeof(struct save_params_success), WEBSOCKET_OP_BINARY);
    }
}

void run_baro_calibration() {
	pthread_t thread_id;
	pthread_create(&thread_id, nullptr, run_baro_calibration_helper, nullptr);
}

void continue_baro_calibration() {
    printf("Continuing barometer calibration\n");

    should_continue = true;
}

static void *run_baro_calibration_helper(void *arg) {
    int pipefd[2];

    if (pipe(pipefd) == -1) {
        perror("pipe");
        exit(1);
    }


	pid_t pid;

    char *argv[] = {
        "/usr/bin/baro_temp_cal",
        "-n", "1250",
        "-i", "1",
        "-c", "4",
        NULL
    };

    pid = fork();

    if (pid == 0) {
        // forward stdout of the calibration to a new pipe
        dup2(pipefd[1], STDOUT_FILENO);
        dup2(pipefd[0], STDIN_FILENO);

        close(pipefd[0]); 
        close(pipefd[1]); 

        execv(argv[0], argv); // run the calibration
    }

    else if (pid > 0) {

        pthread_t thread_id;
        pthread_t thread_id2;
    
        int* stdout_pipe_ptr = new int(pipefd[0]);
        int* stdin_pipe_ptr = new int(pipefd[1]);

        pthread_create(&thread_id, nullptr, read_baro_cal_progress, stdout_pipe_ptr); // thread to read the progress
        pthread_create(&thread_id2, nullptr, wait_for_continue_command, stdin_pipe_ptr);


        int status;
        waitpid(pid, &status, 0);
        pthread_join(thread_id, nullptr); // this thread finishes when there's no more output from the calibration
        delete stdout_pipe_ptr;
        delete stdin_pipe_ptr;


        get_baro_params(); // get the parameters after the calibration

    }


    return 0;
}


void read_baro_cal_progress(void* args) {
    if (args == nullptr) {
        fprintf(stderr, "Error reading baro calibration progress\n");
        return; 
    }

    int pipe_fd = *((int *) args);
    char buffer[31]; // Allocate 11 bytes to accommodate 10 chars + null terminator
    ssize_t bytes_read;
    int start_pos;
    int end_pos;

    int start_pos_square_bracket = -1;
    int end_pos_square_bracket = -1;

    while ((bytes_read = read(pipe_fd, buffer, 30)) > 0) {

        if (strstr(buffer, "Enter") != NULL || strstr(buffer, "stress") != NULL) {
            if (strstr(buffer, "stress") != NULL) {
                should_continue = true;
            }
            send_baro_calibration_stage(CALIBRATION_STAGE_HIT_CONTINUE);
        }

        start_pos = -1;
        end_pos = -1;

        start_pos_square_bracket = -1; 
        end_pos_square_bracket = -1;

        buffer[bytes_read] = '\0'; // Null-terminate the read data

        if (!done_reaching_temp) {
            // Check if the VOXL's too hot
            for (int i = 0; i < bytes_read; ++i) {
                if (buffer[i] == '[') {
                    start_pos_square_bracket = i;
                    break;
                }
            }

            if (start_pos_square_bracket != -1) {
                for (int i = start_pos + 1; i < bytes_read; ++i) {
                    if (buffer[i] == ']') {
                        end_pos_square_bracket = i;
                        break;
                    }
                }
            }
            
            // Extract and send it to the frontend
            if (start_pos_square_bracket != -1 && end_pos_square_bracket != -1) {
                int temp = 0;
                for (int i = start_pos_square_bracket + 1; i < end_pos_square_bracket; i++) {
                    if (isdigit(buffer[i])) {
                        temp = (temp * 10) + (buffer[i] - '0'); 
                    }
                }
                send_baro_calibration_stage(CALIBRATION_STAGE_TOO_HOT, temp);
            }
        }

        // Find the start of the number (position of '(')
        for (int i = 0; i < bytes_read; ++i) {
            if (buffer[i] == '(') {
                start_pos = i;
                break;
            }
        }

        // Find the end of the number (position of ')')
        if (start_pos != -1) {
            for (int i = start_pos + 1; i < bytes_read; ++i) {
                if (buffer[i] == ')') {
                    end_pos = i;
                    break;
                }
            }
        }

        // Extract and send it to the frontend
        if (start_pos != -1 && end_pos != -1) {
            done_reaching_temp = true;
            int number = 0;
            for (int i = start_pos + 1; i < end_pos; i++) {
                if (isdigit(buffer[i])) {
                    number = (number * 10) + (buffer[i] - '0'); 
                }
            }
            send_baro_calibration_progress(number);
        }
    }
}

void wait_for_continue_command(void* args) {

    if (args == nullptr) {
        fprintf(stderr, "Error sending barometer continue command\n");
        return; 
    }

    std::string input_to_child = "\n"; 

    int stdin_pipe_fd = *((int *) args);

    while (true) {
        usleep(5000);

        if (should_continue) {
            for (size_t i = 0; i < 10; i++) { // make 10 attempts to send a newline character
                if (write(stdin_pipe_fd, input_to_child.c_str(), input_to_child.length()) == -1) {
                    fprintf(stderr, "Error writing newline to barometer calibration script\n");
                    continue;
                }
                usleep(500000);
            }

            close(stdin_pipe_fd);
            should_continue = false;
            return;
        }
    }
}



std::vector<std::string> split_line(std::string input) {
    // Parses the parameter file seperating vals on tab

    std::vector<std::string> result;
    size_t pos = 0;
    while ((pos = input.find('\t')) != std::string::npos) {
        result.push_back(input.substr(0, pos));
        input.erase(0, pos + 1); 
    }
    result.push_back(input); 
    return result;
}


void get_baro_params() {
    std::ifstream file("/data/px4/param/parameters_baro_tc.cal");

    if (!file.is_open()) {
        fprintf(stderr, "Error: Could not open barometer calibration params file\n");
    }

    struct baro_calibration_params baroCalParamsMsg;
    baroCalParamsMsg.msg_index = 2;

    std::string line;
    size_t i = 0;
    while (getline(file, line)) {
        if (i >= 11) {
            fprintf(stderr, "Error: improperly formatted barometer calibration file\n");
        }

        std::vector<std::string> res = split_line(line);
        baroCalParamsMsg.param_vals[i] = std::stof(res[3]);

        i++;
    }
    send_baro_calibration_params(baroCalParamsMsg);
}
