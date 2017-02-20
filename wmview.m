function wmview(y,gam,tci)
%WMVIEW  Animate passive dynamic walking data
%   WMVIEW(Y, GAM, TCI) animates the passive dynamic data in Y for slope angle
%   GAM and collision indices TCI.
%   
%   See also: SIMPWM, FULLWM, ACTUWM.

%   Andrew D. Horchler, horchler @ gmail . com, Created 7-7-04
%   Revision: 1.1, 5-1-16


% Leg length
L = 1.5;

% Position of stance foot
xst = 0;
yst = 0;

% Position of hip
xm = xst-L*sin(y(1,1)-gam);
ym = yst+L*cos(y(1,1)-gam);

% Position of swing foot
xsw = xm-L*sin(y(1,3)-y(1,1)+gam);
ysw = ym-L*cos(y(1,3)-y(1,1)+gam);

% Initialize figure for animation
figure('Color','w','Renderer','zbuffer')
axis([xsw 10.55 -1 1.5*L])
axis off
strobePlot = 0;   % Draw stroboscopic plot: 1
tracePlot = 0;    % Trace path of hip and swing foot: 1 or 2

% Draw first position
slope = line([xsw 10.25],[ysw (xsw-10.25)*tan(gam)]);
set(slope,'Color','k','LineWidth',0.1);
stleg = line([xst xm],[yst ym]);
set(stleg,'Color','k','LineStyle','-');
swleg = line([xsw xm],[ysw ym]);
set(swleg,'Color','b','LineWidth',2);
if tracePlot==1
    % Plot position of hip and swing foot
    line([xm xsw],[ym ysw],...
         'Color','k','LineStyle','none','Marker','.','MarkerSize',1);
end
drawnow             % Force Matlab to draw

flipStride = 1;     % Flag for swing-stance flip
xswold = xsw;
yswold = ysw;

% Animate each stride
for j=1:length(tci)-1
    % On collision switch stance and swing legs
    if j>1
        xst = xsw;
        yst = ysw;
        flipStride = -flipStride;
        if strobePlot==1
            set([stleg swleg],'Visible','off');
        end
    end
    
    t1 = tci(j)+1;
    t2 = tci(j+1);
    for i=t1:t2
        if mod(i,20)==0 || i==t1 || i==t2           % When to draw
            xmold = xm;
            ymold = ym;
            xm = xst-L*sin(y(i,1)-gam);          	% Position of hip
            ym = yst+L*cos(y(i,1)-gam);
            
            if tracePlot>1
                line([xmold xm],[ymold ym],'Color',[0.5 0.5 0.5]);
            end
            
            if flipStride==1 && i>t1
                xswold = xsw;
                yswold = ysw;
            end
            xsw = xm-L*sin(y(i,3)-y(i,1)+gam);    	% Position of swing leg
            ysw = ym-L*cos(y(i,3)-y(i,1)+gam);
            if flipStride==1 && tracePlot==2
                % Trace path of blue leg
                line([xswold xsw],[yswold ysw],'Color',[0.5 0.5 1]);
            end

            if strobePlot~=1
                set([stleg swleg],'Visible','off'); % Clear previous position of legs
            else
                cc = 1-(i-t1)/(t2-t1);           	% Scale leg colors for stroboscopic plot
                set([stleg swleg],'Color',[cc cc cc]);
            end
            
            stleg = line([xst xm],[yst ym]);     	% Draw new position of stance leg
            swleg = line([xsw xm],[ysw ym]);      	% Draw new position of swing leg
            if flipStride==-1
                set(stleg,'Color','b','LineWidth',2);
                set(swleg,'Color','k','LineStyle','-');
            else
                set(stleg,'Color','k','LineStyle','-');
                set(swleg,'Color','b','LineWidth',2);
            end
            if tracePlot==1
                % Trace path of hip and swing foot
                line([xm xsw],[ym ysw],...
                     'Color','k','LineStyle','none','Marker','.','MarkerSize',1);
            end
            drawnow                                	% Force Matlab to draw
        end
    end
end