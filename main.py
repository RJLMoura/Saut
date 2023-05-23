import math
import matplotlib.pyplot as plt
import keyboard
import random

# Constants for the interface dimensions
WIDTH = 800
HEIGHT = 600

# Variables for particle position, heading, and speed
particle_x = WIDTH // 2
particle_y = HEIGHT // 2
particle_heading = 0  # In degrees, 0 degrees is facing right
particle_speed = 10

# Variables to store distances
distances = []

# Generate random number of landmarks and their positions
num_landmarks = random.randint(3, 6)
LANDMARKS = []
for _ in range(num_landmarks):
    landmark_x = random.randint(0, WIDTH)
    landmark_y = random.randint(0, HEIGHT)
    LANDMARKS.append((landmark_x, landmark_y))

# Create the figure and axis for the graph
fig, ax = plt.subplots()

# Function to calculate the Euclidean distance between two points
def calculate_distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Function to update the distance between the particle and landmarks
def update_distances():
    distances.clear()
    for landmark in LANDMARKS:
        distance = calculate_distance(particle_x, particle_y, landmark[0], landmark[1])
        distances.append(distance)

# Function to update the particle's position based on its heading and speed
def update_position():
    global particle_x, particle_y  # Declare as global to modify the global variables
    angle = math.radians(particle_heading)
    delta_x = particle_speed * math.cos(angle)
    delta_y = particle_speed * math.sin(angle)
    particle_x += int(delta_x)
    particle_y += int(delta_y)

# Main loop
while True:
    # Update distances
    update_distances()

    # Print distances
    print("Distances to landmarks:")
    for i, distance in enumerate(distances):
        print(f"Landmark {i+1}: {distance:.2f}")

    # Clear the previous plot
    ax.clear()

    # Plot the particle position
    ax.plot(particle_x, particle_y, 'ro', label='Particle')

    # Plot the landmark positions
    for i, landmark in enumerate(LANDMARKS):
        ax.plot(landmark[0], landmark[1], 'bo', label=f'Landmark {i+1}')

    # Set the axis limits
    ax.set_xlim(0, WIDTH)
    ax.set_ylim(0, HEIGHT)

    # Set the title and labels
    ax.set_title('Active Particle')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')

    # Update the legend
    ax.legend()

    # Display the plot
    plt.pause(0.01)

    # Read keyboard input
    if keyboard.is_pressed('up'):
        update_position()
    elif keyboard.is_pressed('left'):
        particle_heading += 10
    elif keyboard.is_pressed('right'):
        particle_heading -= 10
    elif keyboard.is_pressed('q'):
        break  # Exit the program if 'q' key is pressed

    # Limit particle position within the interface dimensions
    particle_x = max(0, min(WIDTH - 1, particle_x))
    particle_y = max(0, min(HEIGHT - 1, particle_y))

# Close the plot window
plt.close(fig)