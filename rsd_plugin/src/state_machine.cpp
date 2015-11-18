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
  timer = new QTimer(this);
  timer->setSingleShot(true);
  connect(timer, SIGNAL(timeout()), this, SLOT(gripperTimeslot()));
  msgBox.setText("ERROR! The system has incountered an error");
  msgBox.setInformativeText("Press ok to reset the system");
  msgBox.setStandardButtons(QMessageBox::Ok);

}

void state_machine::SetIdle(bool _idle){
    idleMutex.lock();
    if(_idle)
        state = IDLE;
    this->idle = _idle;
    idleMutex.unlock();

}

void state_machine::gripperTimeslot(){
    timeout = true;
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
                  cout << "IDLE" << endl;
                  timeout = false;
                  idleMutex.lock();
                  if(!idle){
                      old_state = state;
                      state = READY;
                      position = INIT;
                  }
                  idleMutex.unlock();
                  break;

              case READY:
                  ROS_DEBUG_STREAM("READY");
                  cout << "READY" << endl;
                  old_state = state;
                  state = MOVING;
                  position = CAMERA;
                  break;

              case CAPTURING_IMAGE:
                  ROS_DEBUG_STREAM("CAPTURING_IMAGE");
                  cout << "CAPTURING_IMAGE" << endl;
                  // Capture video until desired bricks are seen in the image.
                  if(srv_call.brickPresent(0)){ ///<<---0 is the index for the brick color
                      old_state = state;
                      state = STOP_BELT;
                  }
                  break;

              case CHECK_BRICKS:
                  ROS_DEBUG_STREAM("CHECK_BRICKS");
                  cout << "CHECK_BRICKS" << endl;
                  // Get brick positions
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
                  timer->start(1000);
                  old_state = state;
                  state = GRIP_CLOSED;
                  break;

              case GRIP_CLOSED:
                  ROS_DEBUG_STREAM("GRIP_CLOSED");
                  cout << "GRIP_CLOSED" << endl;
                  if( timeout ){
                      timeout = false;
                      old_state = state;
                      state = MOVING;
                      position = BACK_FROM_PICK;
                  }
                  break;

              case CHECK_PICK:
                  ROS_DEBUG_STREAM("CHECK_PICK");
                  cout << "CHECK_PICK" << endl;
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
                  cout << "OPEN_GRIP" << endl;
                  // Send open grip command.
                  srv_call.openGripper();
                  old_state = state;
                  state = GRIP_OPENED;
                  break;

              case GRIP_OPENED:
                  ROS_DEBUG_STREAM("GRIP_OPENED");
                  cout << "GRIP_OPENED" << endl;
                  if( timeout ){
                      timeout = false;
                      old_state = state;
                      state = CHECK_BRICKS;
                  }
                  break;

              case MOVING:
                  ROS_DEBUG_STREAM("MOVING to: " << position);
                  cout << "MOVING" << endl;
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
                  if( true/*msgBox.exec() == QMessageBox::Ok*//* User acknowledge */ ){
                      old_state = state;
                      state = READY;
                  }
                  break;

              default:
                  cout << "default! ERROR!" << endl;
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
