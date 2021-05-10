import imageio as imageio
import numpy as np
import simplejson


def main():
    map_fname = './data/ny_1_heights.png'
    data = imageio.imread(map_fname)

    data = data[:, :, 0]

    heights = np.zeros_like(data)

    for i in range(data.shape[0]):
        for j in range(data.shape[1]):
            if data[i, j] < 168:
                heights[i, j] = 100
            else:
                heights[i, j] = 0

    with open('./data/ny_1_heights.json', 'w') as f:
        simplejson.dump(heights.tolist(), f)


if __name__ == "__main__":
    main()