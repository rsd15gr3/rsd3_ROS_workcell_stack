#include "state_machine.h"

enum States{
    IDLE,
    READY,
    START_BELT,
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

state_machine::state_machine(rw::models::WorkCell::Ptr _wc, rw::kinematics::State _state, Device::Ptr _device) {
  timeout = false;
  quitfromgui = false;
  srv_call = state_machine_srv_calls(_wc,_state,_device);
  idle = true;
  moving = false;
  //old_state = IDLE;
  position = INIT;
  timer = new QTimer(this);
  timer->setSingleShot(true);
  connect(timer, SIGNAL(timeout()), this, SLOT(gripperTimeslot()));
  msgBox.setText("ERROR! The system has incountered an error");
  msgBox.setInformativeText("Press ok to reset the system");
  msgBox.setStandardButtons(QMessageBox::Ok);
}

void state_machine::SetIdle(bool _idle){
    if(_idle)
        state = IDLE;
    else{
        state = READY;
    }

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
    }
    else //Start statemachine again
    {
        state = READY;
    }
}

void state_machine::ErrorAck(){
}

void state_machine::run(){
  while(!quitfromgui) {
      switch(state)
      {
              case IDLE:
                  cout << "IDLE" << endl;
                  break;

              case READY:
                  cout << "READY" << endl;
                  old_state = state;
                  state = MOVING;
                  position = CAMERA;
                  break;

              case START_BELT:
                  cout << "START_BELT" << endl;
                  srv_call.conveyorBelt();
                  old_state = state;
                  state = CAPTURING_IMAGE;
                  break;

              case CAPTURING_IMAGE:
                  cout << "CAPTURING_IMAGE" << endl;
                  if(srv_call.brickPresent(0)){ ///<<---0 is the index for the brick color
                      old_state = state;
                      state = STOP_BELT;
                  }
                  break;

              case STOP_BELT:
                  cout << "STOP_BELT" << endl;
                  srv_call.conveyorBeltStop();
                  old_state = state;
                  state = CHECK_BRICKS;
                  break;

              case CHECK_BRICKS:
                  cout << "CHECK_BRICKS" << endl;
                  // Get brick positions
                  if( srv_call.brickPresent(0)/* There are bricks to be picked. */){///<<---0 is the index for the brick color
                      old_state = state;
                      state = MOVING;
                      position = PICK;
                  }
                  else{
                      old_state = state;
                      state = START_BELT;
                  }
                  break;

              case CLOSE_GRIP:
                  cout << "CLOSE_GRIP" << endl;
                  // Send close grip command.
                  srv_call.closeGripper();
                  timer->start(1000); //start timer
                  old_state = state;
                  state = GRIP_CLOSED;
                  break;

              case GRIP_CLOSED:
                  cout << "GRIP_CLOSED" << endl;
                  if( timeout ){ //wait for timer
                      timeout = false;
                      old_state = state;
                      state = MOVING;
                      position = BACK_FROM_PICK;
                  }
                  break;

              case CHECK_PICK:
                  cout << "CHECK_PICK" << endl;
                  // Capture image.
                  if( srv_call.checkPick() ){ ///TODO: check if we have a brick grap
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
                  cout << "OPEN_GRIP" << endl;
                  timer->start(1000);
                  srv_call.openGripper();
                  old_state = state;
                  state = GRIP_OPENED;
                  break;

              case GRIP_OPENED:
                  cout << "GRIP_OPENED" << endl;
                  if( timeout ){
                      timeout = false;
                      old_state = state;
                      state = READY;
                  }
                  break;

              case MOVING:
                  if(!moving){ /* if(Not moving.) then set config */
                      moving = true;

                          switch(position){
                              case INIT:
                                  cout << "MOVING: INIT" << endl;
                                  srv_call.moveTo(Configurations::ConfigInit);
                                  break;

                              case CAMERA:
                                  cout << "MOVING: CAMERA" << endl;
                                  srv_call.moveTo(Configurations::ConfigImgCapture);
                                  break;

                              case BACK_FROM_PICK:
                                  cout << "MOVING: BACK_FROM_PICK" << endl;
                                  srv_call.backOffBrick();
                                  break;

                              case DROP:
                                  cout << "MOVING: DROP" << endl;
                                  srv_call.moveTo(Configurations::ConfigDropoff);
                                  break;

                              case PICK:
                                  cout << "MOVING: PICK" << endl;
                                  srv_call.openGripper();
                                  srv_call.moveToBrickColor(0);///<--0 is the index for the brick color
                                  break;

                              default:
                                  cout << "MOVING: default" << endl;
                                  srv_call.moveTo(Configurations::ConfigInit);
                                  break;
                            }
                  }
                  else  {
                      if(srv_call.closeToConfig()) //if(at config) then chagne state
                      {
                          moving = false;
                          {
                                switch(old_state){
                                    case READY:
                                        cout << "old_state: READY" << endl;
                                        old_state = state;
                                        state = START_BELT;
                                        break;

                                    case CHECK_BRICKS:
                                        cout << "old_state: CHECK_BRICKS" << endl;
                                        old_state = state;
                                        state = CLOSE_GRIP;
                                        break;

                                    case GRIP_CLOSED:
                                        cout << "old_state: GRIPCLOSED" << endl;
                                        old_state = state;
                                        state = CHECK_PICK;
                                        break;

                                    case CHECK_PICK:
                                        cout << "old_state: CHECK_PICK" << endl;
                                        old_state = state;
                                        state = OPEN_GRIP;
                                        break;

                                    default:

                                        break;
                                }
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
