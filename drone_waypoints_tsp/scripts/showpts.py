for k in range(1, 5):
    with open('drone' + str(k) + '_points.txt', 'r') as fp:
        out = fp.read()
        out = out.split(';')
        coords = [map(int, i.split()) for i in out[:-1]]
        path = map(int, out[-1].split())
        path = list(map(lambda x: coords[x], path))
        print("Path for drone" + str(k) + " is:")
        print(path)

