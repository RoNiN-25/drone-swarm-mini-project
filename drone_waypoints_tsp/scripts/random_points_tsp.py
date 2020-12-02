import mlrose_hiive as mlrose
import numpy as np
import sys


# read number of random points to generate
N = int(sys.argv[1])
D = 4

oldout = sys.stdout

for k in range(1, D+1):
    with open('drone' + str(k) + '_points.txt', 'w') as out:
        sys.stdout = out
        # generate N random 3d vectors whose values lie in the range [1, 10)
        coords = np.random.randint(1, 10, size=(N, 3))
        if k == 2:
            coords[:, 0] *= -1
        elif k == 3:
            coords[:, 0] *= -1
            coords[:, 1] *= -1
        elif k == 3:
            coords[:, 1] *= -1
        # compute the distance between each and every point
        dist_list = [(i, j, round(np.linalg.norm(coords[i] - coords[j]), 3))
                     for i in range(N) for j in range(i+1, N)]

        #coords_list = [(1, 1), (4, 2), (5, 2), (6, 4), (4, 4), (3, 6), (1, 5), (2, 3)]
        fitness_coords = mlrose.TravellingSales(distances=dist_list)
        problem_fit = mlrose.TSPOpt(length=len(coords), fitness_fn=fitness_coords, maximize=False)

        best_state, _, _ = mlrose.genetic_alg(problem_fit, random_state=2)

        # jugaad to avoid regex in parsing coords within controller script
        for i in coords:
            print(*i, sep=' ', end=';')
        print(*best_state, sep=' ', end='')
sys.stdout = oldout

