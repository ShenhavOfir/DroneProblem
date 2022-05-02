
VERTICAL = (0.3, 0.3, 0.15, 0.15, 0.1)
HORIZONTAL = (0.15, 0.15, 0.3, 0.3, 0.1)
LAZY = (0.15, 0.15, 0.15, 0.15, 0.4)
UP_LEANING = (0.4, 0.15, 0.15, 0.15, 0.15)
DOWN_LEANING = (0.15, 0.4, 0.15, 0.15, 0.15)
LEFT_LEANING = (0.15, 0.15, 0.4, 0.15, 0.15)
RIGHT_LEANING = (0.15, 0.15, 0.15, 0.4, 0.15)

small_inputs = [

    {
        "map": [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'], ],
        "drones": {'drone 1': (3, 0), },
        "packages": {'package 1': (3, 4),
                     'package 2': (3, 4)},
        "clients": {'Alice': {"location": (0, 0),
                              "packages": ('package 1',),
                              "probabilities": VERTICAL},
                    'Bob': {"location": (2, 0),
                            "packages": ('package 2',),
                            "probabilities": HORIZONTAL}
                    },
        "turns to go": 200
    },

    {
        "map": [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'],
                ['P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'], ],
        "drones": {'drone 1': (3, 0),
                   'drone 2': (2, 4)},
        "packages": {'package 1': (5, 4),
                     'package 2': (5, 4),
                     'package 3': (5, 4),
                     'package 4': (5, 4),
                     'package 5': (5, 4)},
        "clients": {'Alice': {"location": (0, 0),
                              "packages": ('package 1', 'package 2'),
                              "probabilities": RIGHT_LEANING},
                    'Bob': {"location": (2, 3),
                            "packages": ('package 3', 'package 4'),
                            "probabilities": LAZY},
                    'Charlie': {"location": (5, 1),
                                "packages": ('package 5',),
                                "probabilities": VERTICAL},
                    },
        "turns to go": 350
    },


    {
        "map": [['P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'P'], ],
        "drones": {'drone 1': (3, 3)},
        "packages": {'package 1': (2, 2),
                     'package 2': (1, 1)},
        "clients": {'Alice': {"location": (0, 1),
                              "packages": ('package 1', 'package 2'),
                              "probabilities": (0.6, 0.1, 0.1, 0.1, 0.1)}},
        "turns to go": 100
    },

    {
        "map": [['P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P'], ],
        "drones": {'drone 1': (3, 0),
                   'drone 2': (0, 3)},
        "packages": {'package 1': (3, 4),
                     'package 2': (3, 4)},
        "clients": {'Alice': {"location": (0, 0),
                              "packages": ('package 1',),
                              "probabilities": (0.3, 0.3, 0.15, 0.15, 0.1)},
                    'Bob': {"location": (2, 1),
                            "packages": ('package 2',),
                            "probabilities": (0.15, 0.15, 0.3, 0.3, 0.1)}
                    },
        "turns to go": 150
    },
    {
        "map": [['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'I', 'P'],
                ['P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P'],
                ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'I', 'I', 'I', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P', 'I', 'P', 'P', 'P'],
                ['P', 'P', 'P', 'I', 'P', 'P', 'P', 'I', 'P', 'P', 'P', 'P', 'P', 'P', 'P']],
        "drones": {'drone 1': (14, 10),
                   'drone 2': (8, 11),
                   'drone 3': (9, 3),
                   'drone 4': (6, 0),
                   'drone 5': (1, 12),
                   'drone 6': (13, 2)},
        "packages": {'p1': (9, 6),
                     'p2': (9, 6),
                     'p3': (9, 6),
                     'p4': (9, 6),
                     'p5': (9, 6),
                     'p6': (9, 6),
                     'p7': (9, 6),
                     'p8': (9, 6),
                     'p9': (9, 6),
                     'p10': (9, 6),
                     'p11': (9, 6),
                     'p12': (9, 6),
                     'p13': (9, 6),
                     'p14': (9, 6),
                     'p15': (9, 6),
                     'p16': (9, 6),
                     'p17': (9, 6),
                     'p18': (9, 6),
                     'p19': (9, 6),
                     'p20': (9, 6)},
        "clients": {'Alice': {"location": (0, 0),
                              "packages": ('p1', 'p2', 'p3', 'p4', 'p5'),
                              "probabilities": LAZY},
                    'Bob': {"location": (4, 4),
                            "packages": ('p6', 'p7', 'p8', 'p9', 'p10'),
                            "probabilities": VERTICAL},
                    'Charlie': {"location": (8, 11),
                                "packages": ('p11', 'p12', 'p13', 'p14', 'p15'),
                                "probabilities": LEFT_LEANING},
                    'David': {"location": (14, 14),
                              "packages": ('p16', 'p17', 'p18', 'p19', 'p20'),
                              "probabilities": HORIZONTAL},
                    },
        "turns to go": 1250
    }
]
