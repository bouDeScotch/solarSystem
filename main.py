import numpy as np
import pygame
import time

SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 1000
VERTICAL_MARGIN, HORIZONTAL_MARGIN = 100, 100
max_dist = 0
FPS_TARGET = 3000
RADIUS_SCALING = 1
ZOOM = 1
X_SHIFT, Y_SHIFT = 0, 0

class SpaceObject:
    DT = 1e5
    G = 6.67430e-11

    def __init__(self, mass, radius, angle, center_dist, velocity: np.array, color=(255, 255, 255), hist_size=165):
        self.mass = mass
        self.radius = radius
        self.position = np.array([center_dist * np.cos(angle), center_dist * np.sin(angle)])
        self.velocity = velocity
        self.color = color
        self.hist = []
        self.hist_size = hist_size


    def apply_forces(self, other_planets: list['SpaceObject'], iter_per_frame=1):
        for other in other_planets:
            if other == self:
                continue
            dp = other.position - self.position
            r = np.linalg.norm(dp)
            force_val = SpaceObject.G * self.mass * other.mass / r**2
            force = force_val * dp / r
            self.velocity += force / self.mass * SpaceObject.DT / iter_per_frame


    def update(self, other_planets: list['SpaceObject'], iter_per_frame=1):
        self.position += self.velocity * SpaceObject.DT / iter_per_frame
        self.apply_forces(other_planets, iter_per_frame)
        self.hist.append(self.position.copy())
        if len(self.hist) > self.hist_size:
            self.hist.pop(0)


def normalize_pos(pos, max_dist):
    screen_size = max(SCREEN_WIDTH - HORIZONTAL_MARGIN, SCREEN_HEIGHT - VERTICAL_MARGIN)
    before_shift = pos / max_dist * screen_size * ZOOM / 2
    return before_shift[0] + X_SHIFT, before_shift[1] + Y_SHIFT

def draw(display: pygame.Surface, planets):
    global max_dist
    # Reset the screen
    display.fill((0, 0, 0))
    # We will need to normalize the positions of the planets to the screen size
    max_dist = max(*[np.linalg.norm(planet.position) for planet in planets], max_dist)
    for planet in planets:
        # Normalize the position
        x, y = normalize_pos(planet.position, max_dist)
        # Draw the planet
        screen_pos = (int(x + SCREEN_WIDTH / 2), int(y + SCREEN_HEIGHT / 2))
        screen_radius = max(1, int(planet.radius / max_dist * SCREEN_WIDTH * RADIUS_SCALING))
        pygame.draw.circle(display, planet.color, screen_pos, screen_radius)

        # Also draw the path of the planet
        hist_color = (planet.color[0] // 2, planet.color[1] // 2, planet.color[2] // 2)
        hist_screen_radius = max(1, int(planet.radius / max_dist * SCREEN_WIDTH * RADIUS_SCALING / 10))
        for histPos in planet.hist:
            x, y = normalize_pos(histPos, max_dist)
            screen_pos = (int(x + SCREEN_WIDTH / 2), int(y + SCREEN_HEIGHT / 2))
            pygame.draw.circle(display, hist_color, screen_pos, hist_screen_radius)

    pygame.display.flip()

def main():
    global SCREEN_WIDTH, SCREEN_HEIGHT, ZOOM, X_SHIFT, Y_SHIFT
    solar_system = get_solar_system()
    display = pygame.display.set_mode((1000, 1000) , pygame.RESIZABLE)

    mousex, mousey = 0, 0
    pos_clicked_x, pos_clicked_y = 0, 0
    while True:
        start = time.perf_counter_ns() * 1e-9
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return
            if event.type == pygame.VIDEORESIZE:
                SCREEN_WIDTH, SCREEN_HEIGHT = event.dict['size']
                display = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
            # Check if the user moved the mouse
            if event.type == pygame.MOUSEMOTION:
                mousex, mousey = event.pos

            # Check if the user scrolled
            if event.type == pygame.MOUSEBUTTONDOWN:
                if event.button == 4:
                    ZOOM *= 1.1
                elif event.button == 5:
                    ZOOM /= 1.1
                # Check if the user left clicked
                elif event.button == 1:
                    pos_clicked_x, pos_clicked_y = event.pos
            if event.type == pygame.MOUSEBUTTONUP:
                if event.button == 1:
                    X_SHIFT += (event.pos[0] - pos_clicked_x)
                    Y_SHIFT += (event.pos[1] - pos_clicked_y)

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_r:
                    ZOOM = 1
                    X_SHIFT = 0
                    Y_SHIFT = 0
                    DT = 1e5
                # Increase the speed of the simulation using the up arrow key
                if event.key == pygame.K_UP:
                    SpaceObject.DT *= 1.1
                # Decrease the speed of the simulation using the down arrow key
                if event.key == pygame.K_DOWN:
                    SpaceObject.DT /= 1.1


        for i in range(10):
            for planet in solar_system:
                planet.update(solar_system, 10)
        draw(display, solar_system)
        end = time.perf_counter_ns() * 1e-9
        if end - start < 1 / FPS_TARGET:
            time.sleep(1 / FPS_TARGET - (end - start))

def get_solar_system():
    # Sun and planets as SpaceObject instances
    sun = SpaceObject(
        mass=1.989e30,  # Mass of the Sun
        radius=6.96e8,  # Radius of the Sun
        angle=0,
        center_dist=0,  # Distance from Sun to itself
        velocity=np.array([0.0, 0.0]),  # The Sun is stationary in this frame
        color=(255, 255, 0)  # Yellow for the Sun
    )

    mercury = SpaceObject(
        mass=3.285e23,  # Mass of Mercury
        radius=2.4397e6,  # Radius of Mercury
        angle=0,
        center_dist=5.79e10,  # Distance from the Sun
        velocity=np.array([0.0, 47400]),  # Orbital velocity
        color=(169, 169, 169)  # Gray for Mercury
    )

    venus = SpaceObject(
        mass=4.867e24,  # Mass of Venus
        radius=6.0518e6,  # Radius of Venus
        angle=0,
        center_dist=1.082e11,  # Distance from the Sun
        velocity=np.array([0.0, 35020]),  # Orbital velocity
        color=(255, 223, 0)  # Yellowish for Venus
    )

    earth = SpaceObject(
        mass=5.972e24,  # Mass of Earth
        radius=6.371e6,  # Radius of Earth
        angle=0,
        center_dist=1.496e11,  # Distance from the Sun
        velocity=np.array([0.0, 29780]),  # Orbital velocity
        color=(0, 0, 255)  # Blue for Earth
    )

    mars = SpaceObject(
        mass=6.39e23,  # Mass of Mars
        radius=3.3895e6,  # Radius of Mars
        angle=0,
        center_dist=2.279e11,  # Distance from the Sun
        velocity=np.array([0.0, 24070]),  # Orbital velocity
        color=(255, 69, 0)  # Reddish for Mars
    )

    jupiter = SpaceObject(
        mass=1.898e27,  # Mass of Jupiter
        radius=6.9911e7,  # Radius of Jupiter
        angle=0,
        center_dist=7.785e11,  # Distance from the Sun
        velocity=np.array([0.0, 13070]),  # Orbital velocity
        color=(255, 165, 0)  # Orange for Jupiter
    )

    saturn = SpaceObject(
        mass=5.683e26,  # Mass of Saturn
        radius=5.8232e7,  # Radius of Saturn
        angle=0,
        center_dist=1.433e12,  # Distance from the Sun
        velocity=np.array([0.0, 9680]),  # Orbital velocity
        color=(210, 180, 140)  # Tan for Saturn
    )

    uranus = SpaceObject(
        mass=8.681e25,  # Mass of Uranus
        radius=2.5362e7,  # Radius of Uranus
        angle=0,
        center_dist=2.871e12,  # Distance from the Sun
        velocity=np.array([0.0, 6800]),  # Orbital velocity
        color=(173, 216, 230)  # Light blue for Uranus
    )

    neptune = SpaceObject(
        mass=1.024e26,  # Mass of Neptune
        radius=2.4622e7,  # Radius of Neptune
        angle=0,
        center_dist=4.495e12,  # Distance from the Sun
        velocity=np.array([0.0, 5430]),  # Orbital velocity
        color=(0, 0, 128)  # Dark blue for Neptune
    )

    # List of all space objects
    solar_system = [sun, mercury, venus, earth, mars, jupiter, saturn, uranus, neptune]

    return solar_system

def get_3_bodies_system():
    sun = SpaceObject(
        mass=1.989e30,  # Sun's mass
        radius=6.96e8,  # Sun's radius
        angle=0,
        center_dist=0,
        velocity=np.array([0.0, 0.0]),  # Sun is stationary
        color=(255, 255, 0)
    )

    planet1 = SpaceObject(
        mass=5.972e24,  # Earth's mass
        radius=6.371e6,  # Earth's radius
        angle=0,
        center_dist=1.496e11,  # Distance from the Sun
        velocity=np.array([0.0, 29780]),  # Orbital velocity
        color=(0, 0, 255)  # Blue
    )

    planet2 = SpaceObject(
        mass=5.972e24,  # Equal mass to planet1
        radius=6.371e6,  # Same radius
        angle=np.pi,  # Positioned opposite to planet1
        center_dist=1.496e11,  # Same distance from the Sun
        velocity=np.array([0.0, -29780]),  # Opposite orbital velocity
        color=(255, 0, 0)  # Red
    )

    # Add objects to a list
    three_body_system = [sun, planet1, planet2]

    return three_body_system

if __name__ == '__main__':
    main()