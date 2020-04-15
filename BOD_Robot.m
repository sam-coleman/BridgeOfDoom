%Function to Make Neato Drive Across Bridge of Doom
%Authors: Sam Coleman, Ruby Eisenbud, Rishita Sarin
function BOD_Robot()

t = [];
syms t;
%beta_num is our sclaing factor
beta_num = 0.3;
d = 0.235;
assume(t,{'real','positive'});

% this is the equation of the bridge
R = 4*[0.396*cos(2.65*((beta_num*t)+1.4));...
       -0.99*sin((beta_num*t)+1.4);...
       0];

% tangent vector
v = diff(R,t);

% normalized tangent vector 
That = v/norm(v);
dThat = diff(That,t);
%angular velocity
w = cross(That,dThat);

%left and right wheel velocity
VL = simplify(norm(v) - (w(3)*d)/2);
VR = simplify(norm(v) + (w(3)*d)/2);

pub = rospublisher('raw_vel');

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

%place the robot at the origin
bridgeStart = double(subs(R,[t],[0]));
startingThat = double(subs(That,[t],[0]));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);

% time to drive!!

%initialize time variables we will be using
start = rostime('now');
current = rostime('now');
dt = current-start;


while dt.seconds < (3.2/beta_num) - .5  %while time for robot is less than predicted time it will take to cross bridge
    %the -.5 seconds is to account for lag to make sure robot doesn't go too far
    
    %update time variables
    current = rostime('now');
    dt = current - start;
    
    %calculate left and right wheel velocity for time step
    VL_dt = double(subs(VL,t,dt.seconds));
    VR_dt = double(subs(VR,t,dt.seconds));
    
    %send data to robot
    drive = rosmessage(pub);
    drive.Data = [VL_dt,VR_dt];
    send(pub, drive);
    
    %pause to allow enconder data to be collected (using collectDataset)
    pause(0.1)
end
%Stop robot when at end of path
disp('finished loop')
drive.Data = [0, 0];
send(pub, drive);

%Function given to us by teaching team to place robot at origin
function placeNeato(posX, posY, headingX, headingY)
    svc = rossvcclient('gazebo/set_model_state');
    msg = rosmessage(svc);

    msg.ModelState.ModelName = 'neato_standalone';
    startYaw = atan2(headingY, headingX);
    quat = eul2quat([startYaw 0 0]);

    msg.ModelState.Pose.Position.X = posX;
    msg.ModelState.Pose.Position.Y = posY;
    msg.ModelState.Pose.Position.Z = 1.0;
    msg.ModelState.Pose.Orientation.W = quat(1);
    msg.ModelState.Pose.Orientation.X = quat(2);
    msg.ModelState.Pose.Orientation.Y = quat(3);
    msg.ModelState.Pose.Orientation.Z = quat(4);

    % put the robot in the appropriate place
    ret = call(svc, msg);
end
end
