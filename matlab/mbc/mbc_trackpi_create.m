function this = mbc_trackpi_create(rightPoints, leftPoints, optPoints, size)
% this = mbc_trackpi_create(rightPoints, leftPoints, optPoints, size) creates a new track from curbs
%                                  
%   rightPoints - waypoints of right curb
%   leftPoints - waypoints of left curb
%   optPoints - waypoints of ideal line
%   size - track size
%   this - the new track
%
%
% MODBAS CAR mbc
% Copyright (c) 2024, Frank Traenkle

if ~exist('size', 'var')
    size = [];
end
rightPp = mbc_trackpi_spline(rightPoints);
leftPp = mbc_trackpi_spline(leftPoints);
centerPoints = (rightPoints + leftPoints) * 0.5;
centerPp = mbc_trackpi_spline(centerPoints);
rightLanePoints = (rightPoints * 0.75 + leftPoints * 0.25);
rightLanePp = mbc_trackpi_spline(rightLanePoints);
leftLanePoints = (rightPoints * 0.25 + leftPoints * 0.75);
leftLanePp = mbc_trackpi_spline(leftLanePoints);
optPp = mbc_trackpi_spline(optPoints, 0.1);

this = struct('right', rightPp, ...
    'left', leftPp, ...
    'center', centerPp, ...
    'lanes', [ rightLanePp , leftLanePp ], ...
    'opt', optPp, ...
    'size', size, ...
    'periodic', true, ...
    'type', 76);
end

% figure(1); clf;
% %fnplt(ppnew, 'k.');
% hold on;
% fnplt(track.right.pp, 'r');
% plot(rightPoints(1,:), rightPoints(2,:), 'k.');
% fnplt(track.left.pp, 'g');
% plot(leftPoints(1,:), leftPoints(2,:), 'k.');
% fnplt(track.center.pp, 'c');
% plot(track.center.points(2,:), track.center.points(3,:), 'k.');
% quiver(leftPoints(1,:), leftPoints(2,:), ...
%     rightPoints(1,:)-leftPoints(1,:), rightPoints(2,:)-leftPoints(2,:), 'off', 'ShowArrowHead', 'off');
% xlim([-0.1 , totalSize(1)]); ylim([0 , totalSize(2)]);
% axis equal;