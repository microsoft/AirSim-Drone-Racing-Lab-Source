# from PythonClient import airsimdroneracinglab
from time import time
import random
import time
import airsimdroneracinglab as adrl
from airsimdroneracinglab import Pose, Quaternionr, Vector3r


# destroy if needed
def destroy_things(client):
    all_objects = client.simListSceneObjects()
    all_objects = list(set(all_objects))  # remove duplicates
    cones = [thing for thing in all_objects if "cone_" in thing]
    cylinders = [thing for thing in all_objects if "cylinder_" in thing]
    walls = [thing for thing in all_objects if "wall_" in thing]

    [client.simDestroyObject(thing) for thing in cones]
    [client.simDestroyObject(thing) for thing in cylinders]
    [client.simDestroyObject(thing) for thing in walls]
    time.sleep(1.0)


def spawn_walls(client):
    # spawn some walls
    cur_x = random.uniform(-10, 0)

    for idx in range(10):
        client.simSpawnObject(
            "wall_" + str(idx).zfill(3),
            "Wall",
            pose=Pose(Vector3r(cur_x + random.uniform(0, 1), 0, 0), Quaternionr()),
            scale=Vector3r(2.25, 100, 25),
        )
        cur_x += -20
    time.sleep(1.0)


def spawn_cylinders(client):
    # spawn some cylinders
    cur_x = random.uniform(0, 10)

    for idx in range(10):
        client.simSpawnObject(
            "cylinder_" + str(idx).zfill(3),
            "CylinderSpawn",
            pose=Pose(
                Vector3r(cur_x + random.uniform(0, 2), cur_x + random.uniform(0, 2), 0),
                Quaternionr(),
            ),
            scale=Vector3r(3.0, 0.5, 15.0),
        )
        cur_x += 10
    time.sleep(1.0)


def spawn_cones(client):
    # spawn some cones
    cur_x = random.uniform(10, 20)

    for idx in range(10):
        client.simSpawnObject(
            "cone_" + str(idx).zfill(3),
            "ConeSpawn",
            pose=Pose(
                Vector3r(
                    cur_x + random.uniform(0, 2),
                    1.2 * (cur_x + random.uniform(0, 3)),
                    -5 + random.uniform(0, 3),
                ),
                Quaternionr(),
            ),
            scale=Vector3r(4.75, 4.75, 15.0),
        )
        cur_x += 15
    time.sleep(1.0)


def main():
    client = adrl.MultirotorClient()
    client.confirmConnection()

    for idx in range(10):
        client.simLoadLevel("TrainEmpty")
        input("ready camera")
        destroy_things(client)
        spawn_walls(client)
        spawn_cones(client)
        spawn_cylinders(client)


if __name__ == "__main__":
    main()
