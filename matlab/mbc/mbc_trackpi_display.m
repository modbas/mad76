function this = mbc_trackpi_display(this, dx)
% this = mbc_track_display(this, dx, area) calculates the splines of
% a track and displays the track.
%
%   this - the track. The return value is the original track plus the
%          new splines for center line, left line and right line.
%   dx - the step size of the arc length
%   area - the display area (argument of axis)
%
%
% MODBAS CAR mbc
% Copyright (c) 2015, Frank Traenkle

config = mbc_config();
figure(1);
clf;
set(gca, 'Color', config.figure.bgcolor);
hold on;
grid on;
fnplt(this.opt.pp, config.track.ideallinecolor);
fnplt(this.right.pp, config.track.rightcurbcolor);
fnplt(this.left.pp, config.track.leftcurbcolor);
fnplt(this.center.pp, config.track.centercolor);
if config.track.debug
    plot(this.right.points(2,:), this.right.points(3,:), 'ro');
    plot(this.left.points(2,:), this.left.points(3,:), 'go');
    plot(this.center.points(2,:), this.center.points(3,:), 'co');
    plot(this.opt.points(2,:), this.opt.points(3,:), 'mo');
end
for l = this.lanes
    fnplt(l.pp, config.track.lanecolor);
end
axis(this.size);
axis equal;
end