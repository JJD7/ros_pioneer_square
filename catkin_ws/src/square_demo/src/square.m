rosshutdown
rosinit

while(true)
    %intialize publisher
    pub = rospublisher('/cmd_vel');
    %initialize the message
    msg = rosmessage('geometry_msgs/Twist');
    %populate the message
    msg.Linear.X=0.1;
    %publish
    send(pub,msg);
    
    % wait for some time
    pause(5);

    % rotate in place by making linear.x = 0 and angular.z to some non-zero value
    % publish new velocity
    msg.Linear.X=0.0;
    msg.Angular.Z=0.78;
    send(pub,msg);
    % wait for 90 degrees
    pause(2);
    

    %repeat
end
