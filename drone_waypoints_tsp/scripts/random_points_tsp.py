import mlrose_hiive as mlrose
import numpy as np
import sys

# read number of random points to generate
N = int(sys.argv[1])

# generate N random 3d vectors whose values lie in the range [1, 10)
coords = np.random.randint(1, 10, size=(N, 3))
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

