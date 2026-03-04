#ifndef ACTUATOR_TEST_H
#define ACTUATOR_TEST_H

extern int motorVals[4];

#define TestActuatorsClaimString "/test-actuators"

void ActuatorTestCallback(struct mg_connection *c, int ev, void *ev_data, void *fn_data);
static void *run(void *arg);
void send_motor_val(int motorNum, float motorVal);

#endif