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
  ilde = true;
}

void state_machine::SetIdle(bool _ilde){
    if(idle)
    {
        idleMutex.lock();
        idle = _idle;
        state = IDLE;
        idleMutex.unlock();
    }
    else
    {
        idleMutex.lock();
        idle = _idle;
        idleMutex.unlock();
    }
}

void state_machine::quitNow(){
  quitfromgui = true; }

void state_machine::autoControlEnabled(bool _autoenabled){
    if(_autoenabled)
    {
        state = IDLE;
        idle = true;
    }
    else //Start statemachine again
    {
        emit moveToInit();
        ilde = false;
    }
}

void state_machine::run(){
  while(!quitfromgui) {    
      switch(state){
              case IDLE:
                  ROS_DEBUG_STREAM("IDLE");
                  timeout = false;
                  idleMutex.lock();
                  if(!ilde){
                      state = READY;
                  }
                  idleMutex.unlock();
                  break;

              case READY:
                  ROS_DEBUG_STREAM("READY");
                  state = MOVING;
                  position = CAMERA;
                  break;

              case CAPTURING_IMAGE:
                  ROS_DEBUG_STREAM("CAPTURING_IMAGE");
                  // Capture video until bricks are seen in the image.
                  //emit?? or call srv by it self?
                  if( /* There are bricks in the image. */){
                      state = STOP_BELT;
                  }
                  break;

              case CHECK_BRICKS:
                  ROS_DEBUG_STREAM("CHECK_BRICKS");
                  // Get brick positions
                  //emit?? or call srv by it self?
                  if( /* There are bricks to be picked. */){
                      state = MOVING;
                      position = PICK;
                  }
                  else{
                      state = READY;
                  }
                  break;

              case CLOSE_GRIP:
                  ROS_DEBUG_STREAM("CLOSE_GRIP");
                  // Send close grip command.
                  emit closeGripper();
                  gripperTimer = /* n.createTimer(ros::Duration(1.0), gripperTimeCallback); */
                  state = GRIP_CLOSED;
                  break;

              case GRIP_CLOSED:
                  ROS_DEBUG_STREAM("GRIP_CLOSED");
                  if( timeout ){
                      timeout = false;
                      state = MOVING;
                      position = BACK_FROM_PICK;
                  }
                  break;

              case CHECK_PICK:
                  ROS_DEBUG_STREAM("CHECK_PICK");
                  // Capture image.
                  if( /* There is a brick picked. */ ){
                      state = MOVING;
                      position = DROP;
                  }
                  else{
                      state = CHECK_BRICKS;
                  }
                  break;

              case OPEN_GRIP:
                  ROS_DEBUG_STREAM("OPEN_GRIP");
                  // Send open grip command.
                  emit openGripper();
                  gripperTimer = /* n.createTimer(ros::Duration(1.0), gripperTimeCallback); */
                  state = GRIP_OPENED;
                  break;

              case GRIP_OPENED:
                  ROS_DEBUG_STREAM("GRIP_OPENED");
                  if( timeout ){
                      timeout = false;
                      state = CHECK_BRICKS;
                  }
                  break;

              case MOVING:
                  ROS_DEBUG_STREAM("MOVING to: " << position);
                  if( /* Not moving. */ ){
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
                              ///What brick color should we pick??
                              break;

                          default:
                              // Go to init.
                              emit moveToInit();
                              break;
                      }
                  }
                  else{
                      if( /* Moving done. */ ){
                          switch(old_state){
                              case READY:
                                  state = CAPTURING_IMAGE;
                                  break;

                              case CHECK_BRICKS:
                                  state = CLOSE_GRIP;
                                  break;

                              case GRIP_CLOSED:
                                  state = CHECK_PICK;
                                  break;

                              case CHECK_PICK:
                                  state = OPEN_GRIP;
                                  break;

                              default:
                                  state = IDLE;
                                  break;
                          }
                      }
                  }
                  break;

              case ERROR:
                  if( /* User acknowledge */ ){
                      state = READY;
                  }
                  break;

              default:
                  state = ERROR;
                  break;
          }
    }
  if (!quitfromgui) {
    ROS_INFO("ROS-Node Terminated\n"); 
}
}
//http://answers.ros.org/question/53234/processing-an-image-outside-the-callback-function/
