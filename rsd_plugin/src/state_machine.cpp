#include "state_machine.h"

enum States{
    IDLE,
    READY,
    CAPTURING_IMAGE,
    STOP_BELT,
    CHECK_BRICKS,
    CLOSE_GRIP,
    GRIP_CLOSED,
    CHECK_PICK,
    OPEN_GRIP,
    GRIP_OPENED,
    MOVING,
    ERROR
};

enum Positions{
    INIT,
    CAMERA,
    BACK_FROM_PICK,
    DROP,
    PICK
};

state_machine::state_machine() {
  timeout = false;
  quitfromgui = false;
  idle = true;
  state = IDLE;
  old_state = IDLE;
  position = INIT;
}

void state_machine::SetIdle(bool _idle){
    idleMutex.lock();
    if(_idle)
        state = IDLE;
    this->idle = _idle;
    idleMutex.unlock();

}

void state_machine::quitNow(){
  quitfromgui = true; }

void state_machine::autoControlEnabled(bool _autoenabled){
    if(_autoenabled)
    {
        state = IDLE;
        idleMutex.lock();
        idle = true;
        idleMutex.unlock();
    }
    else //Start statemachine again
    {
        emit moveToInit();
        idleMutex.lock();
        idle = false;
        idleMutex.unlock();
    }
}

void state_machine::run(){
  while(!quitfromgui) {
      switch(state){
              case IDLE:
                  ROS_DEBUG_STREAM("IDLE");
                  timeout = false;
                  idleMutex.lock();
                  if(!idle){
                      old_state = state;
                      state = READY;
                  }
                  idleMutex.unlock();
                  break;

              case READY:
                  ROS_DEBUG_STREAM("READY");
                  old_state = state;
                  state = MOVING;
                  position = CAMERA;
                  break;

              case CAPTURING_IMAGE:
                  ROS_DEBUG_STREAM("CAPTURING_IMAGE");
                  // Capture video until desired bricks are seen in the image.
                  if(srv_call.brickPresent(0)){ ///<<---0 is the index for the brick color
                      old_state = state;
                      state = STOP_BELT;
                  }
                  break;

              case CHECK_BRICKS:
                  ROS_DEBUG_STREAM("CHECK_BRICKS");
                  // Get brick positions
                  //emit?? or call srv by it self?
                  if( srv_call.brickPresent(0)/* There are bricks to be picked. */){///<<---0 is the index for the brick color
                      old_state = state;
                      state = MOVING;
                      position = PICK;
                  }
                  else{
                      old_state = state;
                      state = READY;
                  }
                  break;

              case CLOSE_GRIP:
                  ROS_DEBUG_STREAM("CLOSE_GRIP");
                  // Send close grip command.
                  srv_call.closeGripper();
                  gripperTimer = //n.createTimer(ros::Duration(1.0), gripperTimeCallback);
                  old_state = state;
                  state = GRIP_CLOSED;
                  break;

              case GRIP_CLOSED:
                  ROS_DEBUG_STREAM("GRIP_CLOSED");
                  if( timeout ){
                      timeout = false;
                      old_state = state;
                      state = MOVING;
                      position = BACK_FROM_PICK;
                  }
                  break;

              case CHECK_PICK:
                  ROS_DEBUG_STREAM("CHECK_PICK");
                  // Capture image.
                  if( true /* There is a brick picked. */ ){ ///TODO: check if we have a brick grap
                      old_state = state;
                      state = MOVING;
                      position = DROP;
                  }
                  else{
                      old_state = state;
                      state = CHECK_BRICKS;
                  }
                  break;

              case OPEN_GRIP:
                  ROS_DEBUG_STREAM("OPEN_GRIP");
                  // Send open grip command.
                  srv_call.openGripper();
                  gripperTimer = /* n.createTimer(ros::Duration(1.0), gripperTimeCallback); */
                  old_state = state;
                  state = GRIP_OPENED;
                  break;

              case GRIP_OPENED:
                  ROS_DEBUG_STREAM("GRIP_OPENED");
                  if( timeout ){
                      timeout = false;
                      old_state = state;
                      state = CHECK_BRICKS;
                  }
                  break;

              case MOVING:
                  ROS_DEBUG_STREAM("MOVING to: " << position);
                  if(!srv_call.robotMoving() ){ /* if(Not moving.) */
                      switch(position){
                          case INIT:
                              // Go to init position.
                              emit moveToInit();
                              break;

                          case CAMERA:
                              // Go to camera position.
                              emit moveToImgCapture();
                              break;

                          case BACK_FROM_PICK:
                              // Move up.
                              emit backOffBrick();
                              break;

                          case DROP:
                              // Go to drop position.
                              emit moveToDropoff();
                              break;

                          case PICK:
                              // Go to the picking position.
                              emit moveToBrick(0); ///<--0 is the index for the brick color
                              break;

                          default:
                              // Go to init.
                              emit moveToInit();
                              break;
                      }
                  }
                  else{
                      if( true/* Moving done. */ ){ ///TODO: check configuration is correct
                          switch(old_state){
                              case READY:
                                  old_state = state;
                                  state = CAPTURING_IMAGE;
                                  break;

                              case CHECK_BRICKS:
                                  old_state = state;
                                  state = CLOSE_GRIP;
                                  break;

                              case GRIP_CLOSED:
                                  old_state = state;
                                  state = CHECK_PICK;
                                  break;

                              case CHECK_PICK:
                                  old_state = state;
                                  state = OPEN_GRIP;
                                  break;

                              default:
                                  old_state = state;
                                  state = IDLE;
                                  break;
                          }
                      }
                  }
                  break;

              case ERROR:
                  QMessageBox msgBox;
                  msgBox.setText("ERROR! The system has incountered an error in state: " + to_string(state));
                  msgBox.setInformativeText("Press ok to reset the system");
                  msgBox.setStandardButtons(QMessageBox::Ok);
                  int ret = msgBox.exec();
                  if( ret == QMessageBox::Ok/* User acknowledge */ ){
                      old_state = state;
                      state = READY;
                  }
                  break;

              default:
                  old_state = state;
                  state = ERROR;
                  break;
          }
    }
  if (!quitfromgui) {
    ROS_INFO("ROS-Node Terminated\n"); 
}
}
//http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
