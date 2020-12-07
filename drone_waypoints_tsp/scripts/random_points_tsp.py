import mlrose_hiive as mlrose
import numpy as np
import sys


# read number of random points to generate
N = int(sys.argv[1])
# the number of drones
D = 4

oldout = sys.stdout

for k in range(1, D+1):
    
    np.random.seed(k)
    # generate N random 3d vectors whose values lie in the range [1, 10)
    coords = np.random.randint(1, 10, size=(N, 3))
    print(coords)
    if k == 2:
        coords[:, 0] *= -1
    elif k == 3:
        coords[:, 0] *= -1
        coords[:, 1] *= -1
    elif k == 4:
        coords[:, 1] *= -1

    # compute the distance between each and every point
    dist_list = [(i, j, round(np.linalg.norm(coords[i] - coords[j]), 3))
                 for i in range(N) for j in range(i+1, N)]

    # define the fitness object and run the solver
    fitness_coords = mlrose.TravellingSales(distances=dist_list)
    problem_fit = mlrose.TSPOpt(length=len(coords), fitness_fn=fitness_coords, maximize=False)

    best_state, _, _ = mlrose.genetic_alg(problem_fit, random_state=2)

    with open('drone' + str(k) + '_points.txt', 'w') as out:
        # jugaad to avoid regex in parsing coords within controller script
        sys.stdout = out
        for i in coords:
            print(*i, sep=' ', end=';')
        print(*best_state, sep=' ', end='')
    sys.stdout = oldout

