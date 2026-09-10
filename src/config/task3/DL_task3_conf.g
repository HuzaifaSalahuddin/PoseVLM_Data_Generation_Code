world: {},
plane(world): { shape: ssBox, size: [10, 10, 0.1, 0.02], color: [0.3, 0.3, 0.3], contact: 1, mass: 10, inertia: [13.3417, 13.3417, 26.6667] },

table_top(world):       {X  : [0, -0.2, 0.5, 1, 0, 0, 0], shape:box     , size:[1, 0.4, 0.05] , color: [0.996, 0.478, 0.211], contact: 1, mass: 1}
table_leg_1(table_top): {rel: [0.45, -0.15, -0.3, 1, 0, 0, 0], shape:cylinder, size:[0.6, 0.025] , color: [0.996, 0.478, 0.211], contact: 1, mass: 1}
table_leg_2(table_top): {rel: [0.45, 0.15,  -0.3, 1, 0, 0, 0], shape:cylinder, size:[0.6, 0.025] , color: [0.996, 0.478, 0.211], contact: 1, mass: 1}
table_leg_3(table_top): {rel: [-0.45, -0.15, -0.3, 1, 0, 0, 0], shape:cylinder, size:[0.6, 0.025] , color: [0.996, 0.478, 0.211], contact: 1, mass: 1}
table_leg_4(table_top): {rel: [-0.45, 0.15, -0.3, 1, 0, 0, 0], shape:cylinder, size:[0.6, 0.025] , color: [0.996, 0.478, 0.211], contact: 1, mass: 1}

cam_frame_0(world): { rel: [0, 2, 2,  0.00000000,   0.00000000,  -0.92387953,   0.38268344 ], shape: marker, size: [0.001], width: 240, height: 240, focalLength: 2, zRange: [0.5, 3] },
cam_frame_1(world): { rel: [0, -2, 2,    0.38268343,  -0.92387953,   0.00000000,   0.00000000 ], shape: marker, size: [0.001], width: 240, height: 240, focalLength: 2, zRange: [0.5, 3] },
cam_frame_2(world): { rel: [-2, 0.1, 2,   0.27059805,  -0.65328148,   0.65328148,  -0.27059805 ], shape: marker, size: [0.001], width: 240, height: 240, focalLength: 2, zRange: [0.5, 3]  },
cam_frame_3(world): { rel: [2, 0.1, 2,   0.27059805,  -0.65328148,  -0.65328148,   0.27059805 ], shape: marker, size: [0.001], width: 240, height: 240, focalLength: 2, zRange: [0.5, 3]  },
cam_frame_4(world): { rel: [0, 0, 4,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.001], width: 240, height: 240, focalLength: 2, zRange: [0.5, 3]  },
cam_hmap(world): { rel: [2, 0.1, 2,   0.27059805,  -0.65328148,  -0.65328148,   0.27059805 ], shape: marker, size: [0.001], width: 240, height: 240, focalLength: 2, zRange: [0.5, 3]  },