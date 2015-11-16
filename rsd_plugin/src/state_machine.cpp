#include "state_machine.h"
#include <math.h>

state_machine::state_machine() {
  ROS_INFO("Connected to roscore");

  quitfromgui = false; }

void state_machine::quitNow(){
  quitfromgui = true; }

void state_machine::run(){
  while(!quitfromgui) {

    }
  if (!quitfromgui) {
    ROS_INFO("ROS-Node Terminated\n"); 
}
}
//http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
