function [sys,x0]=segbotanim(t,x,u,flag,ts);
%pendubotanim S-function for animating the motion of the Pendubot.

%   Ned Gulley, 6-21-93
%   Copyright (c) 1990-1998 by The MathWorks, Inc. All Rights Reserved.
%   $Revision: 5.6 $
%   Editted by Dan Block

global PendAnim2

if flag==2,
  u(1) = u(1) - pi;  
  shh = get(0,'ShowHiddenHandles');
  set(0,'ShowHiddenHandles','on');

  if any(get(0,'Children')==PendAnim2),
    if strcmp(get(PendAnim2,'Name'),'Segbot Animation'),
      set(0,'currentfigure',PendAnim2);
      hndl=get(gca,'UserData');
      hndl10 = hndl(:,10);
      hndl9 = hndl(:,9);
      hndl8 = hndl(:,8);
      hndl7 = hndl(:,7);
      hndl6 = hndl(:,6);
      hndl5 = hndl(:,5);
      hndl4 = hndl(:,4);
      hndl3 = hndl(:,3);
      hndl2 = hndl(:,2);
      hndl = hndl(:,1);
      b=1.8;
      xBFP=(-b)*sin(u(1)); yBFP=(-b+1.0)*cos(u(1));
      
       
      x=[0 xBFP];
      y=[-1.0 yBFP];
      
      % 'EraseMode' has been taken out, no longer supported after R2014b
      % see http://www.mathworks.com/help/matlab/graphics_transition/how-do-i-replace-the-erasemode-property.html
      set(hndl,'XData',x,'YData',y, ...
           'LineWidth',20, ... % 'EraseMode','normal', ...
           'Marker','.', ...
           'MarkerSize',18);
      set(hndl2,'XData',x(1),'YData',y(1),'linestyle','none','color','k','marker','.','markersize',125);
      rad_line = 0.4;
      set(hndl3,'XData',[x(1),x(1)-rad_line*sin(u(1)+u(2))],'YData',[y(1),y(1)-rad_line*cos(u(1)+u(2))],'color','w','LineWidth',2);
      set(hndl4,'XData',[-0.45*u(2),-0.45*u(2)],'YData',[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
      set(hndl5,'XData',[2.0-(0.45*u(2)),2.0-(0.45*u(2))],'YData',[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
      set(hndl6,'XData',[-2.0-(0.45*u(2)),-2.0-(0.45*u(2))],'YData',[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
      set(hndl7,'XData',[4.0-(0.45*u(2)),4.0-(0.45*u(2))],'YData',[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
      set(hndl8,'XData',[-4.0-(0.45*u(2)),-4.0-(0.45*u(2))],'YData',[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
      set(hndl9,'XData',[6.0-(0.45*u(2)),6.0-(0.45*u(2))],'YData',[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
      set(hndl10,'XData',[-6.0-(0.45*u(2)),-6.0-(0.45*u(2))],'YData',[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);

      drawnow;
    end
  end
  
  set(0,'ShowHiddenHandles',shh);

  sys=[];

elseif flag==0,
  % Initialize the figure for use with this simulation
  [PendAnim2 PendAnim2Axes] = animinit('Segbot Animation');	  
   if ishghandle(PendAnim2)
      % bring figure to foreground
      figure(PendAnim2);
  end
   
  axis(PendAnim2Axes,[-2 2 -3 2]);
  hold(PendAnim2Axes, 'on'); 
  
  % Set up the geometry for the problem
  % SFP=Space Fixed Pivot
  % BFP=Body Fixed Pivot
  b=1.8;
  xSFP=0; ySFP=-1.0;
  xMC=0; yMC=b-1.0;
  x=[xSFP xMC];
  y=[ySFP yMC];
  % add the body
  legh = .8;
  bodyw = 1.6;
  legw = 1.3;
  bodyh = 1;
  mount = 0.6;
  set(0,'currentfigure',PendAnim2);
  plot(PendAnim2Axes, [-3,3],[-1.45,-1.45],'linestyle','-','color',[.4 .4 .4],'LineWidth',7);
%  plot(PendAnim2Axes,[-mount/2,-mount/2,mount/2,mount/2],[-mount/2,mount/2,mount/2,-mount/2],'linestyle','-','color',[0 0 0],'LineWidth',2);
  %% puts a marker on the central pivot 
  plot(PendAnim2Axes, x(1), y(1), 'Marker' , 'o' , ...
  'MarkerEdgeColor' , 'r' , ...
  'MarkerFaceColor' , 'k' , ...
  'MarkerSize' ,10);
  hndl=plot(PendAnim2Axes,x,y, ...
           'LineWidth',20, ...           %'EraseMode','normal', ...
           'Marker','.', ...
           'MarkerSize',18);
           
  plot(PendAnim2Axes, x(1), y(1), 'Marker' , '.' , ... 
  'MarkerEdgeColor' , 'k' , ... 
  'MarkerSize' ,5); 
  hndl2 = plot(PendAnim2Axes,x(1),y(1),'linestyle','none','color','k','marker','.','markersize',125);
  rad_line = 0.25;
  hndl3 = plot(PendAnim2Axes,[x(1),x(1)+rad_line*sin(-pi)],[y(1),y(1)-rad_line*cos(-pi)],'color','w');
  hndl4 = plot(PendAnim2Axes,[0.45,0.45],[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
  hndl5 = plot(PendAnim2Axes,[2.45,2.45],[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
  hndl6 = plot(PendAnim2Axes,[-1.55,-1.55],[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
  hndl7 = plot(PendAnim2Axes,[4.45,4.45],[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
  hndl8 = plot(PendAnim2Axes,[-3.55,-3.55],[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
  hndl9 = plot(PendAnim2Axes,[6.45,6.45],[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
  hndl10 = plot(PendAnim2Axes,[-5.55,-5.55],[-1.45,-1.8],'color',[.4 .4 .4],'LineWidth',4);
  
  set(PendAnim2Axes,'DataAspectRatio',[1 1 1]);
  set(PendAnim2Axes,'UserData',[hndl,hndl2,hndl3,hndl4,hndl5,hndl6,hndl7,hndl8,hndl9,hndl10]);
  

  sys=[0 0 0 2 0 0];
  x0=[];
  str = [];
  ts  = [-1, 0];
  % specify that the simState for this s-function is same as the default
  simStateCompliance = 'DefaultSimState';
  

end

