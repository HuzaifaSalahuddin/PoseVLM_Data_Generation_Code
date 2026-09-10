world: {},
plane(world): { shape: ssBox, size: [10, 10, 0.1, 0.02], color: [0.3, 0.3, 0.3], contact: 1, mass: 10, inertia: [13.3417, 13.3417, 26.6667] },
marker(world): { shape: marker, size: [1, 1, 1], color: [0.3, 0.3, 0.3], contact: 1, mass: 10, inertia: [13.3417, 13.3417, 26.6667] },
shelf(world): { shape: marker, rel: [0, 0, 0, 1, 0, 0, 0], size: [0.01] },
shelf_center_l(shelf): { rel: [0, -0.58, 0.7, -1, 0, 0, 0], shape: box, size: [0.9, 0.4, 0.03, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.0134083, 0.0409083, 0.0541667] },
shelf_back(shelf): { rel: [0, -0.79, 0.40, -1, 0, 0, 0], shape: box, size: [0.9, 0.03, 1.7, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.175283, 0.216042, 0.0409083] },
shelf_front(shelf): { rel: [0, -0.39, 0.35, -1, 0, 0, 0], shape: box, size: [0.9, 0.03, 0.7, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.175283, 0.216042, 0.0409083] },

## To create the table!
table(world): { shape: marker, rel: [0, -0.28, 0, -0.7071068, 0, 0, 0.7071068], size: [0.01] },
table_center(table): { rel: [0, -0.05, 0.6, -1, 0, 0, 0], shape: box, size: [0.4, 0.4, 0.03, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.0134083, 0.0409083, 0.0541667] }, # Original one: -1,0,0,0 Left: -0.7071068, 0, 0, -0.7071068
table_back(table): { rel: [0, -0.26, 0.3, -1, 0, 0, 0], shape: box, size: [0.4, 0.03, 0.6, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.175283, 0.216042, 0.0409083] },
table_front(table): { rel: [0, 0.16, 0.3, -1, 0, 0, 0], shape: box, size: [0.4, 0.03, 0.6, 0.5], color: [0.996, 0.478, 0.211], contact: -1, mass: 1, inertia: [0.175283, 0.216042, 0.0409083] },

goal(shelf_center_l): {rel: [0, 0, 0.10, 1, 0, 0, 0], shape:marker, size: [0.1]}

box(shelf): { X: [0, 0.3, 0.15, 1, 0, 0, 0], joint: rigid, shape: box, size: [0.06, 0.2, 0.2, 0.5], color: [0, 1, 1], contact: 1, mass: 0.1, inertia: [0.000666667, 0.000363333, 0.000363333] },

cam_frame_0(world): { rel: [0, 2, 2,  0.00000000,   0.00000000,  -0.92387953,   0.38268344 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3] },
cam_frame_1(world): { rel: [0, -2, 2,    0.38268343,  -0.92387953,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3] },
cam_frame_2(world): { rel: [-2, 0.1, 2,   0.27059805,  -0.65328148,   0.65328148,  -0.27059805 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3]  },
cam_frame_3(world): { rel: [2, 0.1, 2,   0.27059805,  -0.65328148,  -0.65328148,   0.27059805 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3]  },
cam_frame_4(world): { rel: [0, 0, 3,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3]  },
cam_frame_5(world): { rel: [-1.5, 1.5, 2,   0.17337590,  -0.35808816,   0.85149587,  -0.34156764], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3] },
cam_frame_6(world): { rel: [1.5, 1.5, 2,   0.17987373,  -0.37692368,  -0.84218296,   0.34103055 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3] },
cam_frame_7(world): { rel: [-1.5, -1.5, 2,   0.34935254,  -0.85318381,   0.35729011,  -0.14957931 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3] },
cam_frame_8(world): { rel: [1.5, -1.5, 2,   0.34606604,  -0.85506437,  -0.35627669,   0.14889642 ], shape: marker, size: [0.1], width: 480, height: 480, focalLength: 2, zRange: [0.5, 3] },
