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

bool timeout;
ros::Timer gripperTimer;

void gripperTimeCallback(const ros::TimerEvent&){
	timeout = true;
}

void state_machine(){
	switch(state){
		case IDLE:
			ROS_DEBUG_STREAM("IDLE");
			timeout = false;
			if(/* qtROS->start() */ ){
				state = READY;
			}
			break;
			
		case READY:
			ROS_DEBUG_STREAM("READY");
			state = MOVING;
			position = CAMERA;
			break;
			
		case CAPTURING_IMAGE:
			ROS_DEBUG_STREAM("CAPTURING_IMAGE");
			// Capture video until bricks are seen in the image.
			if( /* There are bricks in the image. */){
				state = STOP_BELT;
			}
			break;
			
		case CHECK_BRICKS:
			ROS_DEBUG_STREAM("CHECK_BRICKS");
			// Get brick positions
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
						break;
						
					case CAMERA:
						// Go to camera position.
						break;
						
					case BACK_FROM_PICK:
						// Move up.
						break;
						
					case DROP:
						// Go to drop position. 
						break;
						
					case PICK:
						// Go to the picking position.
						break;
						
					default:
						// Go to init.
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
						
					
	}
}