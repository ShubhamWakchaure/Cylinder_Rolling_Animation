clc;clear all;close all;
m = 5 % maxx of the cylinder in kg
R = 35 % Radius of bigger curver surface
r = 2  % Radius of small cylinder rolling 
g = 9.81 % Acceleration due to gravity
j = .5*m*(R-r)^2
X = -15 ;Y = R-r % X & Y coordinate of centre of bigger curved surface
C = (2*g)/(3*(R-r)) % Constant which occurs in differential equation
omega = sqrt(C)  % natural frequency

F = VideoWriter('New.avi'); F.FrameRate = 10; open(F);

%% Time for animation
 t = 0:0.1:30
 
%% for plotting circular arc paramteric equation
  th = 270-20:0.1:270+20
  A = X + R*cosd(th) 
  B = Y + R*sind(th)
  
%% Solving the differential equation for cylinder centre point
 % Initial Condition are @t = 0 theta = 10 degrees
 % @ t = 0 theta_dot = 0.05rad/sec
theta_degree  = 0 % initial theta in degrees
theta_radian = theta_degree*pi/180 % initial theta in radians
theta_dot = 0.1 % initial angular velocity in radian/second
phi = atan(theta_radian*omega/theta_dot) % phase angle
%A_disp = theta_radian/sin(phi) % amplitude using the theta
A_disp = theta_dot/(omega * cos(phi)) % Amplitude using the velocity
Angular_Position = A_disp*sin(omega*t + phi) % Angular position of centre w.r.t time
Angular_Velocity = A_disp*omega*cos(omega*t + phi ) % Angular Veloctiy of centre w.r.t.time
Angular_Acce = - A_disp*omega^2 * sin(omega*t + phi)

%% Solving differential equation for peripheral point on cylinder surface  
Point_Angular_Position  = (R/r)*(Angular_Position) 
Point_Angular_Velocity  = (R/r)*(Angular_Velocity)
Point_Angular_Acce = (R/r)*(Angular_Acce)

%% Calculating the Kinetic And Potential Energy
Pe = m*g*(R-r)*(1-cos(Angular_Position)) % Potential energy
Ke = (3/4)*m*(R-r)^2 * Angular_Velocity.^2 % Kinetic Energy (Linear + Rotational Energy)
Te = Pe + Ke


PAPC =  1.5*pi + Angular_Position -  Point_Angular_Position   % converting the point angular position to single coordinate system
%% running the animation using for loop

for i = 1:length(t)
    x(i) = X + (R-r)*cos(Angular_Position(i) + 1.5*pi)  % X coordinate of cylinder centre
    y(i) = Y + (R-r)*sin(Angular_Position(i) + 1.5*pi) % Y coordinate of cylinder centre
    
    %% calculating the all data points for plotting the full cylinder 
    th = 0:0.1:360 % parametric theta
    a = x(i) + r*cosd(th) % X-coordintate of any point on cylinder
    b = y(i) + r*sind(th) % y-coordintate of any point on cylinsder
    %% Setting the position and resolution of grapical window
    set(gcf, 'Position',  [1,2, 1770,960 ])
    
    %% Plotting the Curved Surface and Cylinder
    plot(a,b,'LineWidth',3) % plotting the  whole cylinder
    hold on
    plot(A,B,'LineWidth',3) % plotting the curverd surface
    
    
    %% %%%%%%%%% Important Note  %%%%%%%% %%
    % Below are three different cases
    %1) plotting the centre point
    %2) Plotting the cylinder peripheral point displacment ,velocity and acceleration (Angular)
    %3) PLotting the Energy i.e. variation of KE And PE
     %Comment two and uncomment one and then run 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %% plotting the centre point
%     % *Most important plots*
%     plot(x(i) ,y(i) ,'o','MarkerSize',3,'Color','g','LineWidth',2.5) %Plotting the centre point
%     plot(x(i) + r*cos(PAPC(i)),y(i) + r*sin(PAPC(i)),'o','MarkerSize',3,'Color','r','LineWidth',2.5) % Plotting the point
%     a1 = plot(t(1:i),15*Angular_Position(1:i), 'b' , 'LineWidth' , 2.5) % Angular Position of Point
%     plot([x(i)  t(i)] ,[ y(i)   y(i) ]  , 'm' , 'LineWidth' , 2.5) % Plotting the horizontal line as reference line
%     a2 = plot(t(1:i),10*Angular_Velocity(1:i), 'g' , 'LineWidth' , 2.5) % Angular velocity of point
%     a3 = plot(t(1:i) , 20*Angular_Acce(1:i)  , 'y' , 'LineWidth' , 2.5)  % Angular Acceleration of point 
%     plot([t(i) t(i)],[-3.5 3.5], 'k','LineWidth',2.5)
%     title('Animation of Cylinder Rolling (Centre Point)','FontSize',20)
%     xlabel('Time(sec)','FontSize',15)
%     ylabel('Paramter' ,'FontSize',15)
%     legend([a1;a2; a3],'Angular Displacemt' ,'Angular Velocity', 'Angular Acceleration','FontSize',20)
%     xticks(0:5:30)
%     yticks(0:1:10)
%      axis([-30, 40 , -7 7]) 
    
    %% Plotting the cylinder peripheral point displacment ,velocity and acceleration (Angular)
%     plot(x(i) ,y(i) ,'o','MarkerSize',3,'Color','g','LineWidth',2.5) %Plotting the centre point
%     plot(x(i) + r*cos(PAPC(i)),y(i) + r*sin(PAPC(i)),'o','MarkerSize',3,'Color','r','LineWidth',2.5) % Plotting the point
%     plot([x(i) + r*cos(PAPC(i)) t(i)] ,[ y(i) + r*sin(PAPC(i))  y(i) + r*sin(PAPC(i))] , 'm','LineWidth',2.5) % Line Connecting the point ang graph
%    a1 =  plot(t(1:i),Point_Angular_Position(1:i),'r','LineWidth',2.5) % Actual Angular Position
%    a2 =  plot(t(1:i),Point_Angular_Velocity(1:i),'g','LineWidth',2.5) % Actual Angular Position
%    a3 =  plot(t(1:i),Point_Angular_Acce(1:i),'b','LineWidth',2.5) % Actual Angular Position
%     plot([t(i) t(i)],[-10 10], 'k','LineWidth',2.5)
%     title('Animation of point on the periphery of cylinder','FontSize',20)
%     legend([a1 ;a2;a3],'AngularPosition','Angular Velocity', 'Angular Acceleration','FontSize',20)
%     axis([-30, 40 , -20 20]) % Setting the axis limits
%     xlabel('Time(sec)','FontSize',15)
%     ylabel('Parameter','FontSize',15)
    
     %% PLotting the Energy i.e. variation of KE And PE
%      plot(x(i) ,y(i) ,'o','MarkerSize',3,'Color','g','LineWidth',2.5) %Plotting the centre point
%      plot(x(i) + r*cos(PAPC(i)),y(i) + r*sin(PAPC(i)),'o','MarkerSize',3,'Color','r','LineWidth',2.5) % Plotting the point
%      plot([x(i) + r*cos(PAPC(i)) t(i)] ,[ y(i) + r*sin(PAPC(i))  y(i) + r*sin(PAPC(i))] , 'r','LineWidth',2.5) % Line Connecting the point ang graph
%      plot([x(i)  t(i)] ,[ y(i)   y(i) ]  , 'g' , 'LineWidth' , 2.5) % Plotting the horizontal line as reference line
%      a1 = plot(t(1:i) ,  0.5*Pe(1:i),'m','LineWidth',2.5) % 0.5 for decreasing the amplitude
%      a2 = plot(t(1:i) ,  0.5*Ke(1:i),'c','LineWidth',2.5) % 0.5 for decreasing the amplitude
%      a3 = plot(t(1:i) ,  0.5*Te(1:i),'b','LineWidth',2.5) % 0.5 for decreasing the amplitude
%      plot([t(i) t(i)],[-5 22], 'k','LineWidth',2.5)
%      title('Variation of KE and PE ','FontSize',20)
%      legend([a1;a2; a3],'Potential Energy' ,'Kinetic Energy', 'Total Energy','FontSize',20)
%      axis([-30, 40 , -8 25]) 
%      xlabel('Time(sec)')
%      ylabel('Energy(J)')
   %% setting the position of axes at the origin 
    ax = gca;
    ax.XAxisLocation = 'origin';
    ax.YAxisLocation = 'origin';
    
    %% Writing in video file 
    writeVideo(F, getframe(figure(1)) );
    %pause(0.1)
    hold off
end

%% Closing the video file
close(F); implay('New.avi');