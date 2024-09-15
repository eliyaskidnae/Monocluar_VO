import imageio
import sys


def save_as_gif(image_files, output_file, duration):
    images = []
    for file in image_files:
        images.append(imageio.imread(file))
    imageio.mimsave(output_file, images, duration=duration)


if __name__ == "__main__":
    image_files = sys.argv[1:-2]
    output_file = sys.argv[-2]
    duration = float(sys.argv[-1])
    save_as_gif(image_files, output_file, duration)
