%%
%% Mini-Auto-Drive
%%
%% Race track with no clothoids
%%
%% Copyright (C) 2017-2024, Frank Traenkle
%%
%% frank.traenkle@hs-heilbronn.de, Hochschule Heilbronn, Germany
%%

a1total = 2.7; % total surface width [ m ]
a2total = 1.8; % total surface height [ m ]
a1boundary = 0.05; % margin [ m ]
a2boundary = 0.05; % margin [ m ]
a1 = a1total - 2 * a1boundary; % total surface width [ m ]
a2 = a2total - 2 * a2boundary; % total surface height [ m ]
P_width = 0.25 * a2; % track P_width [ m ]

track = mbc_track_create(a1boundary + P_width, a2boundary + 0.5 * P_width, 0, [ 0, a1total , 0, a2total ]);
track = mbc_straight_create(track, a1 - 2 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, pi, P_width);
track = mbc_straight_create(track, a1 - 3 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, -pi, P_width);
track = mbc_straight_create(track, a1 - 3 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, pi, P_width);
track = mbc_straight_create(track, a1 - 2 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, 0.5 * pi, P_width);
track = mbc_straight_create(track, a2 - 2 * P_width, P_width);
track = mbc_circle_create(track, 0.5 * P_width, 0.5 * pi, P_width);
track = mbc_track_display(track, 0.1);