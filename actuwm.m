function actuwm(gam,steps)
%ACTUWM  Simulation of passive dynamic walking - actuated Simplest Walking Model
%   ACTUWM simulates the "Simplest Walking Model" passive dynamic walker powered
%   by toe-off impulse and a spring-like torque at hip for eight STEPS with a
%   default slope, GAM, of 0 radians.
%   
%   ACTUWM(GAM) optionally specifies the slope angle, GAM, in radians.
%   
%   ACTUWM(GAM, STEPS) optionally specifies the integer number of STEPS to
%   simulate.
%   
%   See also: SIMPWM, WMVIEW.

%   Based on:
%   
%   [1] Arthur D. Kuo "Energetics of Actively Powered Locomotion Using the
%   Simplest Walking Model," ASME Journal of Biomedical Engineering, Vol. 124,
%   No. 1, pp. 113-120, 2002. http://dx.doi.org/10.1115/1.1427703
%   
%   [2] M. Garcia, A. Chatterjee, A. Ruina, and M. Coleman, "The Simplest
%   Walking Model: Stability, Complexity, and Scaling," ASME Journal of
%   Biomedical Engineering, Vol. 120, No. 2, pp. 281-288, 1998.
%   http://dx.doi.org/10.1115/1.2798313
%   
%   [3] Mario W. Gomes "A Derivation of the Transisition Rule at Heelstrike
%   which appears in the paper 'The Simplest Walking Model: Stability,
%   Complexity, and Scaling' by Garcia et al." pp. 1-3, Oct. 4, 1999.
%   http://ruina.tam.cornell.edu/research/topics/locomotion_and_robotics/simplest_walking/simplest_walking_gomes.pdf

%   Andrew D. Horchler, horchler @ gmail . com, Created 7-7-04
%   Revision: 1.1, 5-1-16


% Gamma: angle of slope (radians), used by integration function
if nargin < 1
    gam = 0;
end

% Step period (seconds), frequency of sinusoidal forcing at hip
tau = 3.84;

% Magnitude of sinusoidal forcing at hip
a = 0;

% Spring constant of spring-like torque at hip
k = -0.08;

% Integration time parameters
if nargin < 2
    steps = 8;  % Number of steps to simulate
end
per = 5;        % Max number of seconds allowed per step

% Initial desired step length
s = 0.4;

% IC constants
alpha = asin(0.5*s);

% Toe-off impulse applied at heelstrike condition
if a == 0
    omega = -1.04*alpha;
    P = -omega*tan(alpha);
else
    et = exp(tau)+exp(-tau);
    omega = -alpha*(2+et)/et;
    c2aet = cos(2*alpha)*et;
    P = (c2aet*alpha-2*omega+c2aet*omega)/(2*sin(2*alpha));
end

% Calculate stable ICs from theoretically determined equations
y0 = [alpha;
      omega;
      2*alpha;
      (1-cos(2*alpha))*omega];

% Initialization
y = [];         % Vector to save states
t = [];         % Vector to save times
tci = 0;        % Collision index vector
h = [0 per];	% Integration period in seconds

% Set integration tolerances, turn on collision detection, add more output points
opts = odeset('RelTol',1e-4,'AbsTol',1e-8,'Refine',30,'Events',@collision);

% Loop to perform integration of a noncontinuous function
for i=1:steps
   [tout,yout] = ode45(@(t,y)f(t,y,gam,a,tau,k),h,y0,opts); % Integrate for one stride
   y = [y;yout];                                         	%#ok<AGROW> % Append states to state vector
   t = [t;tout];                                            %#ok<AGROW> % Append times to time vector
   c2y1 = cos(2*y(end,1));                               	% Calculate once for new ICs
   s2y1p = sin(2*y(end,1))*P;                               % Calculate once for new ICs
   y0 = [-y(end,1);
         c2y1*y(end,2)+s2y1p;
         -2*y(end,1);
         c2y1*(1-c2y1)*y(end,2)+(1-c2y1)*s2y1p];            % Mapping to calculate new ICs after collision
   tci = [tci length(t)];                                   %#ok<AGROW> % Append collision index to collision vector
   h = t(end)+[0 per];                                   	% New time step
   s = [s 2*sin(y0(1))];                                 	%#ok<AGROW> % Append last stride length to stride vector
   if i==10                                              	% After gait has stabilized in a few steps
      %k = k+0.01;                                      	% New spring stiffness
      snew = 0.9*s(1);                                    	% New desired step length
      alpha = asin(0.5*snew);                             	% New parameter
      if a ~= 0
          omega = -alpha*(2+et)/et;                     	% Modified parameter if sinusoidal hip torque
      end
      P = -omega*tan(alpha);                              	% New toe-off impulse
   end
end

% Graph collision map
figure(1)
plot(t,y(:,3)-2*y(:,1))
grid on
xlabel('time ( sqrt(l/g) )')
ylabel('\phi(t)-2\theta(t) (rad.)')

% Graph angular positions - the stride function
figure(2)
hold on
plot(t,y(:,1),'r',t,y(:,3),'b--')
grid on
title('Stride Function')
xlabel('time ( sqrt(l/g) )')
ylabel('\phi(t), \theta(t) (rad.)')

% Graph angular velocities
figure(3)
hold on
plot(t,y(:,2),'r',t,y(:,4),'b--')
grid on
title('Angular Velocities')
xlabel('time ( sqrt(l/g) )')
ylabel('\phi^.(t), \theta^.(t) (rad./sqrt(l/g))')

% Phase plot of phi versus theta
figure(4)
plot(y(:,1),y(:,3))
grid on
title('Phase Portrait')
xlabel('\theta(t) (rad.)')
ylabel('\phi(t) (rad.)')

% Plot Hamiltonian
H = 0.5*y(:,2).*y(:,2)+cos(y(:,1)-gam);
figure(5)
plot(t,H-H(1))
grid on
title('Hamiltonian - Total Energy of System')
xlabel('time ( sqrt(l/g) )')
ylabel('Hamiltonian: H(t)-H(0)')

% Run model animation: mview.m
wmview(y,gam,tci)



function ydot=f(t,y,gam,a,tau,k)
%ODE definition
% y1: theta
% y2: thetadot
% y3: phi
% y4: phidot
% gam: slope of incline (radians)
% a: magnitude of sinusoidal forcing at hip
% tau: frequency of sinusoidal forcing at hip
% k: pring stiffness at hip

% Forcing function
F = a*sin(2*pi/tau*t)+k*y(3);

% First order differential equations for Simplest Walking Model
ydot = [y(2);
        sin(y(1)-gam);
        y(4);
        sin(y(1)-gam)+sin(y(3))*(y(2)*y(2)-cos(y(1)-gam))+F];


function [val,ist,dir]=collision(t,y) %#ok<INUSL>
% Check for heelstrike collision using zero-crossing detection

val = y(3)-2*y(1);  % Geometric collision condition, when = 0
ist = 1;			% Stop integrating if collision found
dir = 1;			% Condition only true when passing from - to +