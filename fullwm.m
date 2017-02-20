function fullwm(gam,B,steps)
%FULLWM  Simulation of passive dynamic walking - full dynamics of compass gait
%   FULLWM simulates the full dynamics of a compass gait passive dynamic walker
%   for eight STEPS with a default slope, GAM, of 0.01 radians and a ratio of
%   foot-to-hip mass, B, of 0.01.
%   
%   FULLWM(GAM) optionally specifies the slope angle, GAM, in radians.
%   
%   FULLWM(GAM, B) optionally specifies the ratio of foot to hip mass, B.
%   
%   FULLWM(GAM, B, STEPS) optionally specifies the integer number of STEPS to
%   simulate.
%   
%   See also: SIMPWM, WMVIEW.

%   Based on:
%   
%   [1] M. Garcia, A. Chatterjee, A. Ruina, and M. Coleman, "The Simplest
%   Walking Model: Stability, Complexity, and Scaling," ASME Journal of
%   Biomedical Engineering, Vol. 120, No. 2, pp. 281-288, 1998.
%   http://dx.doi.org/10.1115/1.2798313
%   
%   [2] Mario W. Gomes "A Derivation of the Transisition Rule at Heelstrike
%   which appears in the paper 'The Simplest Walking Model: Stability,
%   Complexity, and Scaling' by Garcia et al." pp. 1-3, Oct. 4, 1999.
%   http://ruina.tam.cornell.edu/research/topics/locomotion_and_robotics/simplest_walking/simplest_walking_gomes.pdf

%   Andrew D. Horchler, horchler @ gmail . com, Created 7-7-04
%   Revision: 1.1, 5-1-16


% Gamma: angle of slope (radians), used by integration function
if nargin < 1
    gam = 0.01;
end

%B: ratio of foot mass to hip mass
if nargin < 2
    B = 0.01;
end

% Integration time parameters
if nargin < 3
    steps = 8;	% Number of steps to simulate
end
per = 5;        % Max number of seconds allowed per step

% IC constants
Theta00 = 0.970956;
Theta10 = -0.270837;
alpha = -1.045203;
c1 = 1.062895;

% Calculate stable ICs from theoretically determined equations, B<0.01
tgam3 = Theta00*gam^(1/3);
y0 = [tgam3+Theta10*gam;
      alpha*tgam3+(alpha*Theta10+c1)*gam;
      2*(tgam3+Theta10*gam);
      (alpha*tgam3+(alpha*Theta10+c1)*gam)*(1-cos(2*(tgam3+Theta10*gam)))];

% Initialization
y = [];         % Vector to save states
t = [];         % Vector to save times
tci = 0;        % Collision index vector
h = [0 per];	% Integration period in seconds

% Set integration tolerances, turn on collision detection, add more output points
opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30,'Events',@collision);

% Loop to perform integration of a noncontinuous function
for i=1:steps
   [tout,yout] = ode45(@(t,y)f(t,y,gam,B),h,y0,opts);   % Integrate for one stride
   y = [y;yout];                                        %#ok<AGROW> % Append states to state vector
   t = [t;tout];                                        %#ok<AGROW> % Append times to time vector
   c2y1 = cos(2*y(end,1));                              % Calculate once for new ICs
   y0 = [-y(end,1);
         c2y1*y(end,2);
         -2*y(end,1);
         c2y1*(1-c2y1)*y(end,2)];                       % Mapping to calculate new ICs after collision
   tci = [tci length(t)];                               %#ok<AGROW> % Append collision index to collision index vector
   h = t(end)+[0 per];                                  % New integration period 
end

% Graph collision map
figure(1)
plot(t,y(:,3)-2*y(:,1))
grid on
xlabel('time (sec.)')
ylabel('\phi(t)-2\theta(t) (rad.)')

% Graph angular positions - the stride function
figure(2)
hold on
plot(t,y(:,1),'r',t,y(:,3),'b--')
grid on
title('Stride Function')
xlabel('time (sec.)')
ylabel('\phi(t), \theta(t) (rad.)')

% Graph angular velocities
figure(3)
hold on
plot(t,y(:,2),'r',t,y(:,4),'b--')
grid on
title('Angular Velocities')
xlabel('time (sec.)')
ylabel('\phi^.(t), \theta^.(t) (rad./sec.)')

% Phase plot of phi versus theta
figure(4)
plot(y(:,1),y(:,3))
grid on
title('Phase Portrait')
xlabel('\theta(t) (rad.)')
ylabel('\phi(t) (rad.)')

% Plot Hamiltonian
H = (0.5+B-B*cos(y(:,3))).*y(:,2).*y(:,2)+0.5*B*y(:,4).*y(:,4)+(B*cos(y(:,3))-B).*y(:,2).*y(:,4)+(1+B)*cos(y(:,1)-gam)-B*cos(y(:,1)-y(:,3)-gam);
figure(5)
plot(t,H-H(1))
grid on
title('Hamiltonian - Total Energy of System')
xlabel('time (sec.)')
ylabel('Hamiltonian: H(t)-H(0)')

% Run model animation: mview.m
wmview(y,gam,tci)



function ydot=f(t,y,gam,B)  %#ok<INUSL>
% ODE definition
% y1: theta
% y2: thetadot
% y3: phi
% y4: phidot
% gam: slope of incline (radians)
% B: ratio foot mass to hip mass

% Gravity divided by leg length
gl = 1;

% Simplifications to minimize operations
sub1 = 1-cos(y(3));
sub2 = B*gl*sin(y(1)-y(3)-gam);
sub3 = 1+2*B*sub1;
sub4 = B*sin(y(3))*y(4)*(y(4)-2*y(2))+(B+1)*gl*sin(y(1)-gam)-sub2;
sub5 = B*y(2)*y(2)*sin(y(3))+sub2;
sub6 = sub3-B*sub1*sub1;

% First order differential equations
ydot = [y(2);
        (sub1*sub5+sub4)/sub6;
        y(4);
        (sub1*sub4+sub3*sub5/B)/sub6];


function [val,ist,dir]=collision(t,y)   %#ok<INUSL>
% Check for heelstrike collision using zero-crossing detection

val = y(3)-2*y(1);    	% Geometric collision condition, when = 0
ist = 1;				% Stop integrating if collision found
dir = 1;				% Condition only true when passing from - to +