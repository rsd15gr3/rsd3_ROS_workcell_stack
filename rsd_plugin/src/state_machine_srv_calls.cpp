#include "state_machine_srv_calls.h"

state_machine_srv_calls::state_machine_srv_calls(){
}

state_machine_srv_calls::state_machine_srv_calls(rw::models::WorkCell::Ptr _wc, rw::kinematics::State _state, Device::Ptr _device){
    this->wc = _wc;
    this->state = _state;
    this->device = _device;
    this->BrickColorToPick = -1;
    //test
}

bool state_machine_srv_calls::openGripper()
{
    bool res = true;
    wsg_50_common::Move srv;
    srv.request.speed = 100;
    srv.request.width = 40;
    //ROS_INFO("Gripper trying to move to width =%d at speed=%d", (int)srv.request.width,(int)srv.request.speed);
    if (ros::service::call("/wsg_50/move",srv))
    {
      if(srv.response.error > 0) {
          //error from gripper... try ack and move again
          /*log().info() << "Gripper error: " << (int)srv.response.error << "\n" <<
                          "Acknowledgeing and trying to open the gripper again..." << "\n";*/
          std_srvs::Empty AckSrv;
          ros::service::call("/wsg_50/ack",AckSrv);
          if (ros::service::call("/wsg_50/move",srv))
          {
              if(srv.response.error > 0) {
                  //log().info() << "Gripper error: " << (int)srv.response.error << "\n";
                  return false;
              }
              else {
                   //log().info() << "Gripper moved to width = " << (int)srv.request.width << " at speed=" << (int)srv.request.speed << "\n";
                   return true;
              }
          }
          return false;
      } else {
      //   log().info() << "Gripper open" << "\n";
      }
   }
    else
    {
        res = false;
    }
    return res;
}

bool state_machine_srv_calls::closeGripper(bool BrickOnSide, double speedPct)
{
    bool res = true;
    wsg_50_common::Move srv;
    srv.request.speed = speedPct;
    if(BrickOnSide)
        srv.request.width = 30.0;//not calibrated
    else
        srv.request.width = 22.0;

    if (ros::service::call("/wsg_50/move",srv))
    {
      if(srv.response.error > 0) {
          std_srvs::Empty AckSrv;
          ros::service::call("/wsg_50/ack",AckSrv);
          if (ros::service::call("/wsg_50/move",srv))
          {
              if(srv.response.error > 0) {
                  return false;
              }
              else {
                   return true;
              }
          }
          return false;
      } else {
           return true;
      }
   }
    else
    {
        return false;
    }
    return res;
}

bool state_machine_srv_calls::brickPresent(int color)
{
    brick_detection::bricks _brick_srv;
    if(ros::service::call("brick_detector/getBricks",_brick_srv))
    {
        cout << _brick_srv.response << endl;
        if(_brick_srv.response.x.size() > 0)
            {
                for(unsigned int i = 0; i !=_brick_srv.response.x.size(); i++)
                {
                    if(color == _brick_srv.response.type[i])
                        return true;
                }
            }
    }
    return false;
}



bool state_machine_srv_calls::OrderedBrickPresent()
{
    _bricks.clear();
    bool r = false;
    bool b = false;
    bool y = false;
    std::vector<int> PresentOrderedColors;
    this->_bricks = this->getBricks();
    if(!_bricks.empty()){
            for(brick _brick : _bricks){
                for(int color : currentOrder){
                    if(color == _brick.color){
                        //if a orderen color is present, then push it back into the vector (but only once)
                        if(color == 0 && r == false)
                        {
                            r = true;
                            PresentOrderedColors.push_back(_brick.color);
                        }
                        else if(color == 1 && y == false)
                        {
                            y = true;
                            PresentOrderedColors.push_back(_brick.color);
                        }
                        else if(color == 2 && b == false)
                        {
                            b = false;
                            PresentOrderedColors.push_back(_brick.color);
                        }
                    }
                }
            }
    }
    if(!PresentOrderedColors.empty()){ //if the vector is not empty, pick a random color from it
        int randomPresentPrderedColor = PresentOrderedColors[rand() % PresentOrderedColors.size()];
        BrickColorToPick = randomPresentPrderedColor;
        cout << "Found " << BrickColorToPick << " \"brick type\" to brick" << endl;
        return true;

    }
    return false;
}

bool state_machine_srv_calls::robotMoving()
{
    kuka_ros::getIsMoving _isMoving_srv;
    ros::service::call("/KukaNode/IsMoving",_isMoving_srv);
    if(_isMoving_srv.response.isMoving)
        return true;
    else
        return false;
}

bool state_machine_srv_calls::closeToConfig(){
    kuka_ros::getConfiguration config;
    if (ros::service::call("/KukaNode/GetConfiguration",config))
    {
        Q currentq = Q(6,config.response.q[0]*0.017453,config.response.q[1]*0.017453,config.response.q[2]*0.017453,
                config.response.q[3]*0.017453,config.response.q[4]*0.017453,config.response.q[5]*0.017453);
        //double norm2 = lastSetQ.norm2() - currentq.norm2();

        for(unsigned int i = 0; i < currentq.size(); i++)
        {
            double delta = currentq[i] - lastSetQ[i];
            //cout << "delta[" << i <<"]: " << delta << endl;
            if(( delta < -0.001 || 0.001 < delta ))
                return false;
        }

        return true;
    }
    else
        return false;
}

bool state_machine_srv_calls::removeFromOrder(int color)
{
    for(std::vector<int>::iterator it=currentOrder.begin(); it!=currentOrder.end(); ++it)
    {
        if(*it == color)
        {
            currentOrder.erase(it);
            return true;
        }
    }
    return false;
}

bool state_machine_srv_calls::removeLastPickedFromOrder(){
    return removeFromOrder(this->lastPickBrickColor);
}

std::vector<Q> state_machine_srv_calls::JacobianIKSover_50trys_Contrainted(Transform3D<> _target){
    std::vector<Q> solutions;
    for(int i = 0; i<50; i++)
    {
        JacobianIKSolver solver(device, state);
        solver.setCheckJointLimits(true);
        solutions = solver.solve(_target, state);
        if(solutions.size() > 0)
        {
            //remove solutions that break the contraints
            for(std::vector<Q>::iterator it = solutions.begin(); it !=solutions.end(); ++it)
            {
                if(!this->checkQcontraints(*it)){
                    solutions.erase(it);
                }
            }
            //if there is any solutions left, return them
            if(solutions.size() > 0)
                return solutions;
        }
    }
    return solutions;
}

bool state_machine_srv_calls::checkQcontraints(Q q){
    if(q[0] < 1.300 || q[0] > 1.85 || q[3] > 1.2 || q[3] < -1.2 || q[1] > 1.88)
        return false;
    else
        return true;
}

std::vector<Q> state_machine_srv_calls::calulatePickupPath(Transform3D<> start, double m_length){

    std::vector<Q> path;
    std::vector<Q> solutions = this->JacobianIKSover_50trys_Contrainted(start);
    if(solutions.size() > 0)
    {
        path.push_back(solutions.at(0));//add step
    }
    else
    {
        return solutions; //return same error as JacobianIKSolver if it fails
    }

    //calulate transform for moving -0.005m along the Z axis
    /*Vector3D<> PosStepMultipli(0.0, 0.0, -0.005);
    Rotation3D<> RotStepMultipli( 1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 1.0);
    Transform3D<> StepMultipli(PosStepMultipli,RotStepMultipli);

    double DistMoved = 0.000;
    Transform3D<> StepTransform = start;
    Transform3D<> nextStepTransform = start;

    while(DistMoved < m_length-0.006) //keep calulating steps down, until the goal is under 0.011m away
    {
        StepTransform.multiply(StepTransform,StepMultipli,nextStepTransform);
        std::vector<Q> solutions = this->JacobianIKSover_50trys(nextStepTransform);
        StepTransform = nextStepTransform;
        if(solutions.size() > 0)
            path.push_back(solutions.at(0));//add step
        else
        {
            log().info() << "calulatePickupPath: JacobianIKSover_50trys failed!\n"
                         << "Is the disired movement along the z axis out of the robot reach??\n";
            return solutions; //return same error as JacobianIKSolver if it fails
        }
        DistMoved += 0.005;
    }*/



    //calulate final pos configuration
    Vector3D<> PosFinalMultipli(0.0, 0.0, -m_length);
    Rotation3D<> RotFinalMultipli( 1.0, 0.0, 0.0,
                                   0.0, 1.0, 0.0,
                                   0.0, 0.0, 1.0);
    Transform3D<> StepFinalMultipli(PosFinalMultipli,RotFinalMultipli);
    Transform3D<> FinalTransform;
    StepFinalMultipli.multiply(start,StepFinalMultipli,FinalTransform);
    std::vector<Q> solutions2 = this->JacobianIKSover_50trys_Contrainted(FinalTransform);
    if(solutions2.size() > 0)
        path.push_back(solutions2.at(0));//add step
    else
    {
      // log().info() << "calulatePickupPath: JacobianIKSover_50trys failed to find the final configuration!\n"
        //             << "Is the disired movement along the z axis out of the robot reach??\n";
        return solutions2; //return same error as JacobianIKSolver if it fails
    }

    return path;
}

bool state_machine_srv_calls::moveToBrick(double xPos, double yPos,double yRot) {
    if(xPos > 0.101 || xPos < -0.101 || yPos > 0.0751 || yPos < -0.0751 || yRot > 1.581 || yRot < -1.581)
    {
        cout << "pickupBrick: xPos, yPos or yRot is breaking contraints!" << endl;
        return false;
    }
    cout << "pickupBrick: x:" << to_string(xPos) << " y: " << to_string(yPos)  << " Rot: " << to_string(yRot) << endl;
    //get frames and calculate Robot->conveyotbelt trasformation
    Frame* conveyorbeltPickupCenter = wc->findFrame("conveyorBeltPickupCenter");
    Transform3D<> _conveyorbeltTopTransformation;
    _conveyorbeltTopTransformation = conveyorbeltPickupCenter->wTf(state);

    Frame* Robot = wc->findFrame("Robot");
    Transform3D<> _RobotTransformation;
    _RobotTransformation = Robot->wTf(state);

    Transform3D<> _invRobotTransformation = rw::math::inverse(_RobotTransformation);
    Transform3D<> _RobotConveyorTransformation;
    _invRobotTransformation.multiply(_invRobotTransformation,_conveyorbeltTopTransformation,_RobotConveyorTransformation);

    Vector3D<> PosOverBrick(-xPos, -yPos, 0.221); //22.1 cm over frame
    RPY<> RotBrickRPY(-yRot,0.0,0.0);
    Rotation3D<> RotBrick = RotBrickRPY.toRotation3D();
    Transform3D<> OverBrickTransformation(PosOverBrick,RotBrick);
    OverBrickTransformation.multiply(_RobotConveyorTransformation,OverBrickTransformation,OverBrickTransformation);
    pickupPath.clear();//clear the vector, so we dont use on any old data
    pickupPath = this->calulatePickupPath(OverBrickTransformation);
    if(!pickupPath.size() > 0)
    {
        //log().info() << "pickupBrick: Failed to pickup brick!\n";
        return false;
    }
    //Check contraints
    for (Q q : pickupPath )
    {
        if(!checkQcontraints(q))
        {
            cout << "moveToBrick: braking contraints!" << endl;
            //log().info() << "pickupBrick: Failed! Found path breaks contraints!\n";
            return false;
        }
    }
    //move robot
    for(Q q : pickupPath )
    {
        this->moveTo(q);
        //this->SetConfiguration(q); //DO set config
    }

    return true;

}

bool state_machine_srv_calls::moveTo(Q _q){ ///TODO: SEND CONFIG
    if(_q.size() == 6)
    {
        lastSetQ = _q;
        //log().info() << "Setting configuration: " << _q << "\n";
        kuka_ros::setConfiguration _q_srv;
        _q_srv.request.q[0] = _q[0];
        _q_srv.request.q[1] = _q[1];
        _q_srv.request.q[2] = _q[2];
        _q_srv.request.q[3] = _q[3];
        _q_srv.request.q[4] = _q[4];
        _q_srv.request.q[5] = _q[5];
        bool srv_call_sucess = false;
        unsigned int counter = 0;
        while(!srv_call_sucess && counter < 5)
        {
            srv_call_sucess = ros::service::call("/KukaNode/SetConfiguration",_q_srv);
            if(srv_call_sucess == false)
                cout << "Kuka srv call: Failed to set configration!... retrying" << endl;
            counter ++;
        }
        if (srv_call_sucess)
        {
            cout << "Kuka srv call: Configration set" << endl;
            while(!this->closeToConfig()){}
            return true;

        }
        cout << "ERROR! Did not set configuration!" << endl;
        return false;
    }
    else
    {
        ROS_ERROR("Error: Q.size != 6 \n");
        cout << "ERROR! Did not set configuration!" << endl;
        return false;
    }
}

std::vector<brick> state_machine_srv_calls::getBricks()
{
    std::vector<brick> bricks;
    brick_detection::bricks _brick_srv;
    brick_detection::bricks::Request req;
    ros::service::call("/brick_detector/getBricks",_brick_srv);
    cout << _brick_srv.response << endl;
    if(!(_brick_srv.response.x.size() > 0))
    {
        cout << "No bricks found" << endl;
        return bricks;
    }

    if(_brick_srv.response.x.size() > 0)
    {
        for(unsigned int i = 0; i !=_brick_srv.response.x.size(); i++)
        {
            brick tmpBrick;
            tmpBrick.color = _brick_srv.response.type[i];
            tmpBrick.x = _brick_srv.response.x[i];
            tmpBrick.y = _brick_srv.response.y[i];
            tmpBrick.angle = _brick_srv.response.angle[i];
            bricks.push_back(tmpBrick);
        }
    }
    //log().info() << "Found: " << _brick_srv.response.x.size() << " bricks\n";
    return bricks;
}

bool state_machine_srv_calls::moveToOrderedBrick(){
    //std::vector<brick> _bricks = this->getBricks();
    std::vector<brick> _colorbricks;

    //pub all bricks of the type "BrickColorToPick" into a vector
    for(brick _brick : _bricks){
            if(_brick.color == BrickColorToPick){
                _colorbricks.push_back(_brick);
            }
    }
    if(_colorbricks.size()>0)
    {
        //brick random brick
        brick randomBrick = _colorbricks[rand() % _colorbricks.size()];
        cout << "x: " << randomBrick.x << " y: " << randomBrick.y << " angle: " << randomBrick.angle << endl;
        lastPickBrickColor = randomBrick.color; //set lastPickBrickColor (so check brick can verify pick)
        return this->moveToBrick(randomBrick.x,randomBrick.y,randomBrick.angle);
    }
    cout << "WARNING!: no brick of the of type: " << BrickColorToPick << " Found. " <<
            "This should never happen" << endl;
    return false;
}

bool state_machine_srv_calls::moveToBrickColor(int color) {
    lastPickBrickColor = color;
    _bricks = this->getBricks();

    for(brick _brick : _bricks)
    {
        if(_brick.color == color)
        {
            /*log().info() << "Moving to brick: \n"
                         << "Color: " << color << "\n"
                         << "X: " << _brick.x << "\n"
                         << "Y: " << _brick.y << "\n"
                         << "Angle: " << _brick.angle << "\n";*/
            return this->moveToBrick(_brick.x,_brick.y,_brick.angle);
        }
    }
    return false;
}

bool state_machine_srv_calls::checkPick(){
    brick_check::check_brick _check_srv;
    _check_srv.request.type = lastPickBrickColor;
    if(ros::service::call("/brick_check_server/checkBrick", _check_srv))
    {
        cout << "checkPick srv: " << _check_srv.response.picked << endl;
        return _check_srv.response.picked;
    }
    else
    {
        cout << "check brick: faild to call srv!" << endl;
        return false;
    }
}

bool state_machine_srv_calls::checkFroboPresent(){
    brick_check::check_brick _check_srv;
    _check_srv.request.type = 3;
    if(ros::service::call("/brick_check_server/checkBrick", _check_srv))
    {
        cout << "checkPick srv: _check_srv.response.picked" << endl;
        return _check_srv.response.picked;
    }
    else
    {
        cout << "check brick: faild to call srv!" << endl;
        return false;
    }
}

bool state_machine_srv_calls::backOffBrick(){
    if (!this->pickupPath.size() > 0)
    {
        //log().info() << "backOffBrick: Failed! pickupPath vector is empty!!\n";
        return false;
    }
    else
    {
        /*for (unsigned i = this->pickupPath.size(); i-- > 0; )
        {
            this->SetConfiguration(pickupPath.at(i));
        }*/
        this->moveTo(pickupPath.front());
        this->moveTo(Configurations::ConfigCheckPick);
        return true;
    }
}

bool state_machine_srv_calls::conveyorBelt(int speed, bool activate, bool forward){
    //Speed 1: 50Hz, Speed 2: 35Hz
    plc_comm::plc_service _srv;
    _srv.request.action = activate;
    _srv.request.direction = forward;
    if(speed == 1 || speed == 2)
    _srv.request.preDefSpeed = speed;
    else
        return false;

    if(ros::service::call("/operate_PLC",_srv))
        return true;
    else
        cout << "ERROR!" << endl;
        return false;
}

bool state_machine_srv_calls::conveyorBeltStop(){
    plc_comm::plc_service _srv;
    _srv.request.action = false;
    _srv.request.direction = true;
     _srv.request.preDefSpeed = 2;
     if(ros::service::call("/operate_PLC",_srv))
         return true;
     else
         return false;
}
