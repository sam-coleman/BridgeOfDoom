function BOD_Robot()
% Insert any setup code you want to run here

% define u explicitly to avoid error when using sub functions
% see: https://www.mathworks.com/matlabcentral/answers/268580-error-attempt-to-add-variable-to-a-static-workspace-when-it-is-not-in-workspace
t = [];
% u will be our parameter
syms t;
alpha_num = 0.5;
d = 0.235;
assume(t,{'real','positive'});

% this is the equation of the bridge
R = 4*[0.396*cos(2.65*((alpha_num*t)+1.4));...
       -0.99*sin((alpha_num*t)+1.4);...
       0];

% tangent vector
v = diff(R,t);

% normalized tangent vector
That = v/norm(v);
dThat = diff(That,t);
w = cross(That,dThat);

VL = simplify(norm(v) - (w(3)*d)/2);
VR = simplify(norm(v) + (w(3)*d)/2);

pub = rospublisher('raw_vel');

% stop the robot if it's going right now
stopMsg = rosmessage(pub);
stopMsg.Data = [0 0];
send(pub, stopMsg);

bridgeStart = double(subs(R,[t],[0]));
startingThat = double(subs(That,[t],[0]));
placeNeato(bridgeStart(1),  bridgeStart(2), startingThat(1), startingThat(2));

% wait a bit for robot to fall onto the bridge
pause(2);

% time to drive!!

% For simulated Neatos only:
% Place the Neato in the specified x, y position and specified heading vector.

start = rostime('now');
current = rostime('now');

dt = current-start;

%t_array = linspace(0,(3.2/0.3),100);

while dt.seconds < (3.2/0.5)  %while time for robot is less than predicted time it will take to cross bridge
    current = rostime('now');
    dt = current - start;
    
%     R_dt = double(subs(R,t,dt.seconds));
%     v_dt = double(subs(v,t,dt.seconds));
%     That_dt = double(subs(That,t,dt.seconds));
%     dThat_dt = double(subs(dThat,t,dt.seconds));
%     w_dt = double(subs(w,t,dt.seconds));
    VL_dt = double(subs(VL,t,dt.seconds));
    VR_dt = double(subs(VR,t,dt.seconds));
    drive = rosmessage(pub);
    drive.Data = [VL_dt,VR_dt];
    send(pub, drive);
end

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
