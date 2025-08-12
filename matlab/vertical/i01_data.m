%%
%% Mini-Auto-Drive
%%
%% Component Interfaces (Bus Objects)
%%
%% Copyright (C) 2017-2022, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%

%% Model Variant
VariantCom = VARIANT_COM_TYPE.ROS2;
%VariantCom = VARIANT_COM_TYPE.MSG;

%% Maximum number of cars for ROS receive message processing
P_car_cnt = 4;

%% Maximum number of checkpoints for ROS receive message processing
P_cp_cnt = 23;
P_cpid_CameraStart = uint64(1);
P_cpid_CtrlStart = uint64(7);
P_cpid_CtrlPublish = uint64(8);
P_cpid_BehaviorStart = uint64(11);
P_cpid_BehaviorPublish = uint64(12);

%% Bus Objects
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 's';
elems(1).DataType = 'single';
elems(1).Dimensions = [ 2 1 ];
elems(2) = Simulink.BusElement;
elems(2).Name = 'psi';
elems(2).DataType = 'single';
elems(3) = Simulink.BusElement;
elems(3).Name = 'beta';
elems(3).DataType = 'single';
elems(4) = Simulink.BusElement;
elems(4).Name = 'v';
elems(4).DataType = 'single';
elems(5) = Simulink.BusElement;
elems(5).Name = 'x';
elems(5).DataType = 'single';
elems(6) = Simulink.BusElement;
elems(6).Name = 'prob';
elems(6).DataType = 'single';
CAROUTPUTSEXT = Simulink.Bus;
CAROUTPUTSEXT.Elements = elems;

P_caroutputsext = Simulink.Bus.createMATLABStruct('CAROUTPUTSEXT', [], [1 P_car_cnt]);

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 's';
elems(1).DataType = 'single';
elems(1).Dimensions = [ 2 1 ];
elems(2) = Simulink.BusElement;
elems(2).Name = 'psi';
elems(2).DataType = 'single';
elems(3) = Simulink.BusElement;
elems(3).Name = 'beta';
elems(3).DataType = 'single';
elems(4) = Simulink.BusElement;
elems(4).Name = 'v';
elems(4).DataType = 'single';
% center line
elems(5) = Simulink.BusElement;
elems(5).Name = 'cxe'; % center line length
elems(5).DataType = 'single';
elems(6) = Simulink.BusElement;
elems(6).Name = 'cx'; % car pos projected to center line
elems(6).DataType = 'single';
elems(7) = Simulink.BusElement;
elems(7).Name = 'cxd'; % center line pos speed
elems(7).DataType = 'single';
elems(8) = Simulink.BusElement;
elems(8).Name = 'cey'; % lateral deviation
elems(8).DataType = 'single';
elems(9) = Simulink.BusElement;
elems(9).Name = 'cepsi'; % yaw angle deviation
elems(9).DataType = 'single';
elems(10) = Simulink.BusElement;
elems(10).Name = 'ckappa'; % center line curvature
elems(10).DataType = 'single';
% right curb distance
elems(11) = Simulink.BusElement;
elems(11).Name = 'rey';
elems(11).DataType = 'single';
% left curb distance
elems(12) = Simulink.BusElement;
elems(12).Name = 'ley';
elems(12).DataType = 'single';
% reliability
elems(13) = Simulink.BusElement;
elems(13).Name = 'prob';
elems(13).DataType = 'single';
CAROBS = Simulink.Bus;
CAROBS.Elements = elems;

P_carobs = Simulink.Bus.createMATLABStruct('CAROBS', [], [1 P_car_cnt]);

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 's';
elems(1).DataType = 'single';
elems(1).Dimensions = [ 2 1 ];
elems(2) = Simulink.BusElement;
elems(2).Name = 'psi';
elems(2).DataType = 'single';
elems(3) = Simulink.BusElement;
elems(3).Name = 'v';
elems(3).DataType = 'single';
CTRLREFERENCE = Simulink.Bus;
CTRLREFERENCE.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 's';
elems(1).DataType = 'single';
elems(1).Dimensions = [ 2 1 ];
elems(2) = Simulink.BusElement;
elems(2).Name = 'psi';
elems(2).DataType = 'single';
CAROUTPUTS = Simulink.Bus;
CAROUTPUTS.Elements = elems;

% 1 cm track discretization
P_max_breakslen = uint32(200); %968; % 713 % 673;
clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'breakslen';
elems(1).DataType = 'uint32';
elems(2) = Simulink.BusElement;
elems(2).Name = 'points';
elems(2).DataType = 'single';
elems(2).Dimensions = [ 3 double(P_max_breakslen) ];
elems(3) = Simulink.BusElement;
elems(3).Name = 'coefs';
elems(3).DataType = 'single';
elems(3).Dimensions = [ double(P_max_breakslen)*2  4 ];
elems(4) = Simulink.BusElement;
elems(4).Name = 'segments';
elems(4).DataType = 'uint32';
elems(4).Dimensions = [ 1 double(P_max_breakslen) ];
elems(5).Name = 'periodic';
elems(5).DataType = 'boolean';
global SPLINE;
SPLINE = Simulink.Bus;
SPLINE.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'segments';
elems(1).DataType = 'uint8';
elems(1).Dimensions = [ 1 100 ];
elems(2) = Simulink.BusElement;
elems(2).Name = 'periodic';
elems(2).DataType = 'boolean';
elems(3) = Simulink.BusElement;
elems(3).Name = 'vmax';
elems(3).DataType = 'single';
elems(4) = Simulink.BusElement;
elems(4).Name = 'xref';
elems(4).DataType = 'single';
elems(5) = Simulink.BusElement;
elems(5).Name = 'lapcount';
elems(5).DataType = 'uint32';
DRIVEROUTE = Simulink.Bus;
DRIVEROUTE.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'sec';
elems(1).DataType = 'int32';
elems(2) = Simulink.BusElement;
elems(2).Name = 'nsec';
elems(2).DataType = 'int32';
TIMEMSG = Simulink.Bus;
TIMEMSG.Elements = elems;
TIMEMSG.DataScope = 'Exported';
TIMEMSG.HeaderFile = 'TimeMsg';

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'seqctr';
elems(1).DataType = 'uint64';
elems(2) = Simulink.BusElement;
elems(2).Name = 'cpId';
elems(2).DataType = 'uint64';
elems(3) = Simulink.BusElement;
elems(3).Name = 'time';
elems(3).DataType = 'Bus: TIMEMSG';
CHECKPOINTMSG = Simulink.Bus;
CHECKPOINTMSG.Elements = elems;
CHECKPOINTMSG.DataScope = 'Exported';
CHECKPOINTMSG.HeaderFile = 'CheckpointMsg';

P_cp = Simulink.Bus.createMATLABStruct('CHECKPOINTMSG', []);

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'cpseq';
elems(1).DataType = 'Bus: CHECKPOINTMSG';
elems(1).Dimensions = [1 P_cp_cnt];
elems(2) = Simulink.BusElement;
elems(2).Name = 'health';
elems(2).DataType = 'boolean';
CHECKPOINTSEQUENCEMSG = Simulink.Bus;
CHECKPOINTSEQUENCEMSG.Elements = elems;
CHECKPOINTSEQUENCEMSG.DataScope = 'Exported';
CHECKPOINTSEQUENCEMSG.HeaderFile = 'CheckpointSequenceMsg';

P_cpseq = Simulink.Bus.createMATLABStruct('CHECKPOINTSEQUENCEMSG', []);

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'vmax';
elems(1).DataType = 'single';
elems(2) = Simulink.BusElement;
elems(2).Name = 'type';
elems(2).DataType = 'uint8';
ManeuverTypeHalt = uint8(0);
ManeuverTypePark = uint8(1);
ManeuverTypePathFollow = uint8(3);
ManeuverTypeLaneChange = uint8(4);
ManeuverTypeBreakupLaneChange = uint8(5);
elems(3) = Simulink.BusElement;
elems(3).Name = 'xref';
elems(3).DataType = 'single';
elems(4) = Simulink.BusElement;
elems(4).Name = 'lapCount';
elems(4).DataType = 'uint32';
elems(5) = Simulink.BusElement;
elems(5).Name = 'disableLaneMonitor';
elems(5).DataType = 'boolean';
elems(6) = Simulink.BusElement;
elems(6).Name = 'isnew';
elems(6).DataType = 'boolean';
elems(7) = Simulink.BusElement;
elems(7).Name = 'breakslen';
elems(7).DataType = 'uint32';
elems(8) = Simulink.BusElement;
elems(8).Name = 'points';
elems(8).DataType = 'single';
elems(8).Dimensions = [ 3 double(P_max_breakslen) ];
elems(9) = Simulink.BusElement;
elems(9).Name = 'coefs';
elems(9).DataType = 'single';
elems(9).Dimensions = [ double(P_max_breakslen)*2  4 ];
elems(10) = Simulink.BusElement;
elems(10).Name = 'segments';
elems(10).DataType = 'uint32';
elems(10).Dimensions = [ 1 double(P_max_breakslen) ];
elems(11) = Simulink.BusElement;
elems(11).Name = 'periodic';
elems(11).DataType = 'boolean';
elems(12) = Simulink.BusElement;
elems(12).Name = 'cpseq';
elems(12).DataType = 'Bus: CHECKPOINTSEQUENCEMSG';
DRIVEMANEUVER = Simulink.Bus;
DRIVEMANEUVER.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'cmd';
elems(1).DataType = 'uint8';
CarInputsCmdHalt = uint8(0);
CarInputsCmdForward = uint8(1);
CarInputsCmdReverse = uint8(2);
CarInputsCmdSlow = uint8(3);
CarInputsCmdCharge = uint8(4);
elems(2) = Simulink.BusElement;
elems(2).Name = 'pedals';
elems(2).DataType = 'single';
elems(3) = Simulink.BusElement;
elems(3).Name = 'steering';
elems(3).DataType = 'single';
elems(4) = Simulink.BusElement;
elems(4).Name = 'cpseq';
elems(4).DataType = 'Bus: CHECKPOINTSEQUENCEMSG';
CARINPUTS = Simulink.Bus;
CARINPUTS.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'vmax';
elems(1).DataType = 'single';
elems(2) = Simulink.BusElement;
elems(2).Name = 'type';
elems(2).DataType = 'uint8';
ManeuverTypeHalt = uint8(0);
ManeuverTypePark = uint8(1);
ManeuverTypePathFollow = uint8(3);
ManeuverTypeLaneChange = uint8(4);
ManeuverTypeBreakupLaneChange = uint8(5);
elems(3) = Simulink.BusElement;
elems(3).Name = 'xref';
elems(3).DataType = 'single';
elems(4) = Simulink.BusElement;
elems(4).Name = 'lapCount';
elems(4).DataType = 'uint32';
elems(5) = Simulink.BusElement;
elems(5).Name = 'disableLaneMonitor';
elems(5).DataType = 'boolean';
elems(6) = Simulink.BusElement;
elems(6).Name = 'isnew';
elems(6).DataType = 'boolean';
MANEUVER = Simulink.Bus;
MANEUVER.Elements = elems;

P_seqctr_max = uint8(3);

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'seqctr';
elems(1).DataType = 'uint8';
elems(1).Description = 'sequence counter';
elems(1).Min = 0;
elems(1).Max = double(P_seqctr_max);
elems(2) = Simulink.BusElement;
elems(2).Name = 'mutex';
elems(2).DataType = 'Enum: MUTEX';
elems(2).Description = 'channel owning mutex';
TOKEN = Simulink.Bus;
TOKEN.Elements = elems;

clear elems;
elems(1) = Simulink.BusElement;
elems(1).Name = 'type';
elems(1).DataType = 'uint8';
elems(1).Description = 'type of faultinjection';
OK = uint8(0);
AlivnessInjection = uint8(1);
DeadlineInjection = uint8(2);
FAULTINJECTION = Simulink.Bus;
FAULTINJECTION.Elements = elems;